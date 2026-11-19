"""Offline LeRobot v3.0 dataset collector for headless ``SimEnv`` rollouts."""

from __future__ import annotations

import json
import math
import shutil
import struct
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Self

import numpy as np

from .env import SimEnv

CODEBASE_VERSION = "v3.0"
CHUNKS_SIZE = 1000
DATA_PATH = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
VIDEO_PATH = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
EPISODES_PATH = "meta/episodes/chunk-000/file-000.parquet"
QUANTILES = (0.01, 0.10, 0.50, 0.90, 0.99)
VALID_UNIT_MODES = ("rad", "so101_norm", "deg")
STATS_IMAGE_EVERY_S = 0.5
DEFAULT_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": [1], "names": None},
    "frame_index": {"dtype": "int64", "shape": [1], "names": None},
    "episode_index": {"dtype": "int64", "shape": [1], "names": None},
    "index": {"dtype": "int64", "shape": [1], "names": None},
    "task_index": {"dtype": "int64", "shape": [1], "names": None},
}


class CollectorError(ValueError):
    """Raised when collector configuration, step inputs, or episode state violate contract."""


def _is_bool(val: Any) -> bool:
    return isinstance(val, (bool, np.bool_))


def _validate_finite_1d(
    values: Any,
    expected_len: int,
    label: str,
) -> list[float]:
    if values is None or _is_bool(values) or isinstance(values, (str, bytes, dict)):
        raise CollectorError(f"{label} must be a 1D numeric sequence of length {expected_len}")

    if isinstance(values, Sequence) and not isinstance(values, np.ndarray):
        for idx, item in enumerate(values):
            if _is_bool(item) or isinstance(item, (Sequence, np.ndarray)) and not isinstance(item, (str, bytes)):
                raise CollectorError(f"{label}[{idx}] must be a numeric scalar, got {item!r}")

    raw = np.asarray(values)
    if np.issubdtype(raw.dtype, np.bool_):
        raise CollectorError(f"{label} must be numeric, not boolean")
    if raw.ndim == 2 and raw.shape[0] == 1:
        raw = raw[0]
    elif raw.ndim != 1:
        raise CollectorError(
            f"{label} must be 1D of length {expected_len} or shape (1, {expected_len}), got shape {raw.shape}"
        )
    if raw.shape[0] != expected_len:
        raise CollectorError(f"{label} length mismatch: expected {expected_len}, got {raw.shape[0]}")

    try:
        arr = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise CollectorError(f"{label} must be numeric: {exc}") from exc

    if not np.all(np.isfinite(arr)):
        raise CollectorError(f"{label} contains non-finite values (NaN or Inf)")
    return [float(x) for x in arr]


def _validate_image(
    image: Any,
    role: str,
    expected_shape: tuple[int, int, int] | None,
) -> np.ndarray:
    if image is None or _is_bool(image) or isinstance(image, (str, bytes, dict)):
        raise CollectorError(f"Missing or invalid camera frame for required role {role!r}")

    arr = np.asarray(image)
    if np.issubdtype(arr.dtype, np.bool_):
        raise CollectorError(f"Camera frame for {role!r} must not be boolean")
    if arr.ndim != 3 or arr.shape[2] != 3 or arr.shape[0] <= 0 or arr.shape[1] <= 0:
        raise CollectorError(
            f"Camera frame for {role!r} must have shape (H, W, 3) with H,W > 0, got {arr.shape}"
        )
    if expected_shape is not None and arr.shape != expected_shape:
        raise CollectorError(
            f"Camera frame shape mismatch for {role!r}: expected {expected_shape}, got {arr.shape}"
        )

    if not np.all(np.isfinite(arr)):
        raise CollectorError(f"Camera frame for {role!r} contains non-finite values (NaN or Inf)")

    if arr.dtype == np.uint8:
        return np.ascontiguousarray(arr)
    if np.issubdtype(arr.dtype, np.integer):
        vmin, vmax = int(arr.min()), int(arr.max())
        if vmin < 0 or vmax > 255:
            raise CollectorError(
                f"Integer camera frame for {role!r} out of [0, 255] range: [{vmin}, {vmax}]"
            )
        return np.ascontiguousarray(arr.astype(np.uint8))
    if np.issubdtype(arr.dtype, np.floating):
        vmin, vmax = float(arr.min()), float(arr.max())
        if vmin < 0.0 or vmax > 255.0:
            raise CollectorError(
                f"Float camera frame for {role!r} out of valid range: [{vmin}, {vmax}]"
            )
        if vmax <= 1.0:
            return np.ascontiguousarray(np.round(arr * 255.0).astype(np.uint8))
        return np.ascontiguousarray(np.round(arr).astype(np.uint8))
    raise CollectorError(f"Unsupported camera frame dtype {arr.dtype!r} for role {role!r}")


# --------------------------------------------------------------------------- video


def pick_vcodec(vcodec: str = "auto") -> str:
    """``auto``: libsvtav1 (LeRobot default) when PyAV has it, else h264."""
    import av

    if vcodec != "auto":
        return vcodec
    for name in ("libsvtav1", "h264"):
        try:
            av.codec.Codec(name, "w")
            return name
        except Exception:
            continue
    raise CollectorError("PyAV has neither libsvtav1 nor an h264 encoder")


@dataclass
class VideoSettings:
    vcodec: str = "auto"
    pix_fmt: str = "yuv420p"
    g: int = 2
    crf: int = 30
    preset: int | None = None

    def options(self) -> dict[str, str]:
        opts = {"g": str(self.g), "crf": str(self.crf)}
        if self.vcodec == "libsvtav1":
            opts["preset"] = str(12 if self.preset is None else self.preset)
        elif self.preset is not None:
            opts["preset"] = str(self.preset)
        return opts


class _VideoWriter:
    """Encode RGB uint8 frames to mp4 with one frame at ``pts = i / fps``."""

    def __init__(self, path: Path, fps: int, settings: VideoSettings, size: tuple[int, int]) -> None:
        import av

        self.path = path
        self.fps = fps
        self.settings = settings
        self.size = size  # (width, height)
        self.frames = 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._out = av.open(str(self.path), "w")
        self._stream = self._out.add_stream(settings.vcodec, rate=fps, options=settings.options())
        self._stream.width, self._stream.height = size
        self._stream.pix_fmt = settings.pix_fmt
        self._closed = False

    def put(self, image: np.ndarray) -> None:
        import av

        if self._closed:
            raise CollectorError("video writer already closed")
        w, h = self.size
        vf = av.VideoFrame.from_ndarray(np.ascontiguousarray(image[..., :3]), format="rgb24")
        frame = vf.reformat(width=w, height=h, format=self.settings.pix_fmt)
        frame.pts = self.frames
        for packet in self._stream.encode(frame):
            self._out.mux(packet)
        self.frames += 1

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            for packet in self._stream.encode(None):
                self._out.mux(packet)
        finally:
            self._out.close()


def probe_video(path: Path) -> dict[str, Any]:
    import av

    with av.open(str(path)) as container:
        s = container.streams.video[0]
        rate = s.average_rate or s.base_rate
        return {
            "video.height": s.height,
            "video.width": s.width,
            "video.codec": s.codec_context.codec.canonical_name,
            "video.pix_fmt": s.pix_fmt,
            "video.fps": round(rate.numerator / rate.denominator),
            "video.channels": 3,
            "has_audio": len(container.streams.audio) > 0,
            "frames": s.frames,
        }


# --------------------------------------------------------------------------- stats


def _stats_image(image: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    arr = image
    if max(h, w) >= 300:
        step = max(1, int((w if w > h else h) / 150))
        arr = arr[::step, ::step]
    return np.ascontiguousarray(arr[..., :3])


def feature_stats(data: np.ndarray) -> dict[str, np.ndarray]:
    """Per-feature stats over axis 0: min/max/mean/std/count/q01..q99."""
    arr = np.asarray(data, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr[:, None]
    out: dict[str, np.ndarray] = {
        "min": arr.min(axis=0),
        "max": arr.max(axis=0),
        "mean": arr.mean(axis=0),
        "std": arr.std(axis=0),
        "count": np.array([len(arr)], dtype=np.int64),
    }
    for q in QUANTILES:
        out[f"q{int(round(q * 100)):02d}"] = np.quantile(arr, q, axis=0)
    return out


def image_stats(images: Sequence[np.ndarray]) -> dict[str, np.ndarray]:
    """Per-channel stats of RGB uint8 images (H, W, 3), scaled to [0, 1], shape (3, 1, 1)."""
    px = np.concatenate([im.reshape(-1, 3) for im in images]).astype(np.float64) / 255.0
    stats = feature_stats(px)
    stats.pop("count")
    out = {k: v.reshape(3, 1, 1) for k, v in stats.items()}
    out["count"] = np.array([len(images)], dtype=np.int64)
    return out


def aggregate_stats(per_episode: list[dict[str, dict[str, np.ndarray]]]) -> dict[str, dict[str, np.ndarray]]:
    """LeRobot ``aggregate_stats``: count-weighted mean and quantiles, pooled variance."""
    out: dict[str, dict[str, np.ndarray]] = {}
    for key in per_episode[0]:
        items = [ep[key] for ep in per_episode if key in ep]
        counts = np.stack([s["count"] for s in items]).astype(np.float64)
        total = counts.sum(axis=0)
        means = np.stack([s["mean"] for s in items])
        w = counts.reshape(counts.shape + (1,) * (means.ndim - counts.ndim))
        mean = (means * w).sum(axis=0) / total
        var = ((np.stack([s["std"] for s in items]) ** 2 + (means - mean) ** 2) * w).sum(axis=0) / total
        agg: dict[str, np.ndarray] = {
            "min": np.min(np.stack([s["min"] for s in items]), axis=0),
            "max": np.max(np.stack([s["max"] for s in items]), axis=0),
            "mean": mean,
            "std": np.sqrt(var),
            "count": total.astype(np.int64),
        }
        for q_key in [k for k in items[0] if k.startswith("q")]:
            agg[q_key] = (np.stack([s[q_key] for s in items]) * w).sum(axis=0) / total
        out[key] = agg
    return out


def _jsonable(stats: dict[str, dict[str, np.ndarray]]) -> dict[str, Any]:
    return {k: {s: np.asarray(v).tolist() for s, v in d.items()} for k, d in stats.items()}


# --------------------------------------------------------------------------- parquet writer


def _uvarint(n: int) -> bytes:
    out = bytearray()
    while n >= 0x80:
        out.append((n & 0x7F) | 0x80)
        n >>= 7
    out.append(n)
    return bytes(out)


def _zigzag(n: int) -> int:
    return (n << 1) ^ (n >> 63)


class _CompactWriter:
    """Minimal Thrift TCompactProtocol serializer for Parquet FileMetaData and PageHeader."""

    BOOL_TRUE = 1
    BOOL_FALSE = 2
    I16 = 4
    I32 = 5
    I64 = 6
    DOUBLE = 7
    BINARY = 8
    LIST = 9
    STRUCT = 12

    def __init__(self) -> None:
        self.buf = bytearray()
        self._last_fid: list[int] = [0]

    def field_hdr(self, fid: int, ttype: int) -> None:
        delta = fid - self._last_fid[-1]
        if 1 <= delta <= 15:
            self.buf.append((delta << 4) | ttype)
        else:
            self.buf.append(ttype)
            self.buf.extend(_uvarint(_zigzag(fid)))
        self._last_fid[-1] = fid

    def write_i32(self, fid: int, val: int) -> None:
        self.field_hdr(fid, self.I32)
        self.buf.extend(_uvarint(_zigzag(int(val))))

    def write_i64(self, fid: int, val: int) -> None:
        self.field_hdr(fid, self.I64)
        self.buf.extend(_uvarint(_zigzag(int(val))))

    def write_str(self, fid: int, val: str) -> None:
        self.field_hdr(fid, self.BINARY)
        raw = val.encode("utf-8")
        self.buf.extend(_uvarint(len(raw)))
        self.buf.extend(raw)

    def begin_struct(self, fid: int | None = None) -> None:
        if fid is not None:
            self.field_hdr(fid, self.STRUCT)
        self._last_fid.append(0)

    def end_struct(self) -> None:
        self.buf.append(0)
        self._last_fid.pop()

    def write_list_hdr(self, fid: int, elem_type: int, count: int) -> None:
        self.field_hdr(fid, self.LIST)
        if count < 15:
            self.buf.append((count << 4) | elem_type)
        else:
            self.buf.append(0xF0 | elem_type)
            self.buf.extend(_uvarint(count))

    def raw_i32(self, val: int) -> None:
        self.buf.extend(_uvarint(_zigzag(int(val))))

    def raw_str(self, val: str) -> None:
        raw = val.encode("utf-8")
        self.buf.extend(_uvarint(len(raw)))
        self.buf.extend(raw)


def _rle_bitpack_1bit(bits: Sequence[int]) -> bytes:
    """Encode a 0/1 sequence using Parquet RLE/bit-packing hybrid (bit_width=1)."""
    n = len(bits)
    if n == 0:
        return b""
    n_groups = (n + 7) // 8
    out = bytearray(_uvarint((n_groups << 1) | 1))
    for g in range(n_groups):
        byte = 0
        base = g * 8
        for bit_idx in range(8):
            if base + bit_idx < n and bits[base + bit_idx]:
                byte |= 1 << bit_idx
        out.append(byte)
    return bytes(out)


def _rle_constant(value: int, count: int) -> bytes:
    """Encode ``count`` copies of a constant level (bit_width=1)."""
    if count == 0:
        return b""
    return _uvarint(count << 1) + bytes([value])


@dataclass
class _ParquetColumnSpec:
    name: str
    phys_type: int  # 2=INT64, 4=FLOAT, 5=DOUBLE, 6=BYTE_ARRAY
    is_list: bool
    is_utf8: bool
    num_values: int
    rep_levels: bytes
    def_levels: bytes
    plain_data: bytes

    @property
    def path_in_schema(self) -> list[str]:
        if self.is_list:
            return [self.name, "list", "element"]
        return [self.name]


def _encode_plain_values(values: Any, phys_type: int) -> bytes:
    if phys_type == 4:  # FLOAT
        return np.asarray(values, dtype="<f4").tobytes()
    if phys_type == 5:  # DOUBLE
        return np.asarray(values, dtype="<f8").tobytes()
    if phys_type == 2:  # INT64
        return np.asarray(values, dtype="<i8").tobytes()
    if phys_type == 6:  # BYTE_ARRAY (UTF8)
        out = bytearray()
        for s in values:
            b = str(s).encode("utf-8")
            out.extend(struct.pack("<I", len(b)))
            out.extend(b)
        return bytes(out)
    raise CollectorError(f"Unsupported Parquet physical type {phys_type}")


def _build_parquet_column(name: str, kind: str, data: Any) -> _ParquetColumnSpec:
    # kind: "float32", "int64", "float64", "string", "list_float32", "list_float64", "list_int64", "list_string"
    if kind.startswith("list_"):
        elem_kind = kind[len("list_") :]
        phys_map = {"float32": (4, False), "float64": (5, False), "int64": (2, False), "string": (6, True)}
        phys_type, is_utf8 = phys_map[elem_kind]
        flat_vals: list[Any] = []
        rep_bits: list[int] = []
        def_bits: list[int] = []
        for row in data:
            row_list = list(row)
            if not row_list:
                raise CollectorError(f"Empty list element not supported for column {name!r}")
            for j, item in enumerate(row_list):
                flat_vals.append(item)
                rep_bits.append(0 if j == 0 else 1)
                def_bits.append(1)
        num_values = len(flat_vals)
        rep_raw = _rle_bitpack_1bit(rep_bits)
        def_raw = _rle_constant(1, num_values)
        rep_bytes = struct.pack("<I", len(rep_raw)) + rep_raw
        def_bytes = struct.pack("<I", len(def_raw)) + def_raw
        plain = _encode_plain_values(flat_vals, phys_type)
        return _ParquetColumnSpec(
            name=name,
            phys_type=phys_type,
            is_list=True,
            is_utf8=is_utf8,
            num_values=num_values,
            rep_levels=rep_bytes,
            def_levels=def_bytes,
            plain_data=plain,
        )

    phys_map = {"float32": (4, False), "float64": (5, False), "int64": (2, False), "string": (6, True)}
    phys_type, is_utf8 = phys_map[kind]
    vals = list(data) if kind == "string" else data
    return _ParquetColumnSpec(
        name=name,
        phys_type=phys_type,
        is_list=False,
        is_utf8=is_utf8,
        num_values=len(vals),
        rep_levels=b"",
        def_levels=b"",
        plain_data=_encode_plain_values(vals, phys_type),
    )


def _write_parquet_pure_python(
    path: Path,
    columns: list[tuple[str, str, Any]],
    num_rows: int,
    kv_metadata: dict[str, str] | None = None,
) -> None:
    """Write an uncompressed Apache Parquet v1 file using PLAIN encoding (zero external deps)."""
    specs = [_build_parquet_column(name, kind, values) for name, kind, values in columns]

    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".parquet.tmp")
    chunk_metas: list[tuple[_ParquetColumnSpec, int, int]] = []  # (spec, offset, total_size)

    with open(tmp, "wb") as f:
        f.write(b"PAR1")
        for spec in specs:
            payload = spec.rep_levels + spec.def_levels + spec.plain_data
            pw = _CompactWriter()
            pw.write_i32(1, 0)  # PageType.DATA_PAGE
            pw.write_i32(2, len(payload))  # uncompressed_page_size
            pw.write_i32(3, len(payload))  # compressed_page_size
            pw.begin_struct(5)  # data_page_header
            pw.write_i32(1, spec.num_values)
            pw.write_i32(2, 0)  # Encoding.PLAIN
            pw.write_i32(3, 3)  # Encoding.RLE
            pw.write_i32(4, 3)  # Encoding.RLE
            pw.end_struct()
            pw.end_struct()
            page_bytes = bytes(pw.buf) + payload
            offset = f.tell()
            f.write(page_bytes)
            chunk_metas.append((spec, offset, len(page_bytes)))

        # Build SchemaElements (root + each column)
        schema_elements: list[dict[str, Any]] = [
            {"name": "schema", "num_children": len(specs)},
        ]
        for spec in specs:
            if spec.is_list:
                schema_elements.append(
                    {
                        "name": spec.name,
                        "repetition_type": 0,  # REQUIRED
                        "num_children": 1,
                        "converted_type": 3,  # LIST
                        "logical_list": True,
                    }
                )
                schema_elements.append(
                    {
                        "name": "list",
                        "repetition_type": 2,  # REPEATED
                        "num_children": 1,
                    }
                )
                elem: dict[str, Any] = {
                    "name": "element",
                    "type": spec.phys_type,
                    "repetition_type": 0,  # REQUIRED
                }
                if spec.is_utf8:
                    elem["converted_type"] = 0  # UTF8
                    elem["logical_str"] = True
                schema_elements.append(elem)
            else:
                elem = {
                    "name": spec.name,
                    "type": spec.phys_type,
                    "repetition_type": 0,  # REQUIRED
                }
                if spec.is_utf8:
                    elem["converted_type"] = 0  # UTF8
                    elem["logical_str"] = True
                schema_elements.append(elem)

        mw = _CompactWriter()
        mw.write_i32(1, 1)  # version
        mw.write_list_hdr(2, _CompactWriter.STRUCT, len(schema_elements))
        for se in schema_elements:
            mw.begin_struct()
            if "type" in se:
                mw.write_i32(1, se["type"])
            if "repetition_type" in se:
                mw.write_i32(3, se["repetition_type"])
            mw.write_str(4, se["name"])
            if "num_children" in se:
                mw.write_i32(5, se["num_children"])
            if "converted_type" in se:
                mw.write_i32(6, se["converted_type"])
            if se.get("logical_str"):
                mw.begin_struct(10)  # LogicalType
                mw.begin_struct(1)  # StringType
                mw.end_struct()
                mw.end_struct()
            elif se.get("logical_list"):
                mw.begin_struct(10)  # LogicalType
                mw.begin_struct(3)  # ListType
                mw.end_struct()
                mw.end_struct()
            mw.end_struct()

        mw.write_i64(3, num_rows)

        # RowGroups (1 row group)
        total_byte_size = sum(total_sz for _, _, total_sz in chunk_metas)
        mw.write_list_hdr(4, _CompactWriter.STRUCT, 1)
        mw.begin_struct()  # RowGroup
        mw.write_list_hdr(1, _CompactWriter.STRUCT, len(chunk_metas))
        for spec, offset, total_sz in chunk_metas:
            mw.begin_struct()  # ColumnChunk
            mw.write_i64(2, offset)  # file_offset
            mw.begin_struct(3)  # ColumnMetaData
            mw.write_i32(1, spec.phys_type)
            encs = [3, 0] if spec.is_list else [0]
            mw.write_list_hdr(2, _CompactWriter.I32, len(encs))
            for enc in encs:
                mw.raw_i32(enc)
            p_schema = spec.path_in_schema
            mw.write_list_hdr(3, _CompactWriter.BINARY, len(p_schema))
            for part in p_schema:
                mw.raw_str(part)
            mw.write_i32(4, 0)  # CompressionCodec.UNCOMPRESSED
            mw.write_i64(5, spec.num_values)
            mw.write_i64(6, total_sz)
            mw.write_i64(7, total_sz)
            mw.write_i64(9, offset)  # data_page_offset
            mw.end_struct()  # ColumnMetaData
            mw.end_struct()  # ColumnChunk
        mw.write_i64(2, total_byte_size)
        mw.write_i64(3, num_rows)
        mw.end_struct()  # RowGroup

        if kv_metadata:
            mw.write_list_hdr(5, _CompactWriter.STRUCT, len(kv_metadata))
            for k, v in kv_metadata.items():
                mw.begin_struct()
                mw.write_str(1, k)
                mw.write_str(2, v)
                mw.end_struct()

        mw.write_str(6, "rebot-sim-collector")
        mw.end_struct()

        meta_bytes = bytes(mw.buf)
        f.write(meta_bytes)
        f.write(struct.pack("<I", len(meta_bytes)))
        f.write(b"PAR1")

    tmp.replace(path)


def _has_pyarrow() -> bool:
    try:
        import pyarrow  # noqa: F401
        import pyarrow.parquet  # noqa: F401

        return True
    except ImportError:
        return False


def _write_data_parquet(
    path: Path,
    columns: dict[str, np.ndarray],
    hf_metadata: dict[str, Any],
) -> None:
    if _has_pyarrow():
        import pyarrow as pa
        import pyarrow.parquet as pq

        arrays, fields = [], []
        for key, values in columns.items():
            if values.ndim == 2:
                arr = pa.FixedSizeListArray.from_arrays(
                    pa.array(values.reshape(-1), type=pa.float32()),
                    values.shape[1],
                )
            else:
                arr = pa.array(values)
            arrays.append(arr)
            fields.append(pa.field(key, arr.type))
        schema = pa.schema(fields, metadata={"huggingface": json.dumps(hf_metadata)})
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".parquet.tmp")
        pq.write_table(pa.Table.from_arrays(arrays, schema=schema), tmp)
        tmp.replace(path)
        return

    cols_spec: list[tuple[str, str, Any]] = []
    num_rows = 0
    for key, values in columns.items():
        num_rows = len(values)
        if values.ndim == 2:
            cols_spec.append((key, "list_float32", values.astype(np.float32)))
        elif values.dtype == np.float32:
            cols_spec.append((key, "float32", values))
        elif values.dtype == np.int64:
            cols_spec.append((key, "int64", values))
        else:
            cols_spec.append((key, "float64", values.astype(np.float64)))
    _write_parquet_pure_python(
        path,
        cols_spec,
        num_rows=num_rows,
        kv_metadata={"huggingface": json.dumps(hf_metadata)},
    )


def _pandas_tasks_metadata(pyarrow_version: str = "0.0.0") -> dict[str, Any]:
    return {
        "index_columns": ["task"],
        "column_indexes": [
            {
                "name": None,
                "field_name": None,
                "pandas_type": "unicode",
                "numpy_type": "object",
                "metadata": {"encoding": "UTF-8"},
            }
        ],
        "columns": [
            {
                "name": "task_index",
                "field_name": "task_index",
                "pandas_type": "int64",
                "numpy_type": "int64",
                "metadata": None,
            },
            {
                "name": "task",
                "field_name": "task",
                "pandas_type": "unicode",
                "numpy_type": "object",
                "metadata": None,
            },
        ],
        "attributes": {},
        "creator": {"library": "rebot-sim-collector", "version": pyarrow_version},
        "pandas_version": "2.0.0",
    }


def _write_tasks_parquet(path: Path, tasks: dict[str, int]) -> None:
    items = sorted(tasks.items(), key=lambda kv: kv[1])
    if _has_pyarrow():
        import pyarrow as pa
        import pyarrow.parquet as pq

        pandas_meta = _pandas_tasks_metadata(pa.__version__)
        schema = pa.schema(
            [pa.field("task_index", pa.int64()), pa.field("task", pa.string())],
            metadata={"pandas": json.dumps(pandas_meta)},
        )
        table = pa.table([[i for _, i in items], [t for t, _ in items]], schema=schema)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".parquet.tmp")
        pq.write_table(table, tmp)
        tmp.replace(path)
        return

    pandas_meta = _pandas_tasks_metadata()
    cols = [
        ("task_index", "int64", np.array([i for _, i in items], dtype=np.int64)),
        ("task", "string", [t for t, _ in items]),
    ]
    _write_parquet_pure_python(
        path,
        cols,
        num_rows=len(items),
        kv_metadata={"pandas": json.dumps(pandas_meta)},
    )


def _write_episodes_parquet(path: Path, episodes: list[dict[str, Any]]) -> None:
    if _has_pyarrow():
        import pyarrow as pa
        import pyarrow.parquet as pq

        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".parquet.tmp")
        pq.write_table(pa.Table.from_pylist(episodes), tmp)
        tmp.replace(path)
        return

    keys = list(episodes[0].keys())
    cols: list[tuple[str, str, Any]] = []
    for k in keys:
        vals = [ep[k] for ep in episodes]
        sample = vals[0]
        if isinstance(sample, list):
            if sample and isinstance(sample[0], str):
                cols.append((k, "list_string", vals))
            elif k.endswith("/count"):
                cols.append((k, "list_int64", vals))
            else:
                cols.append((k, "list_float64", [np.asarray(v, dtype=np.float64).ravel().tolist() for v in vals]))
        elif isinstance(sample, int) and not isinstance(sample, bool):
            cols.append((k, "int64", np.array(vals, dtype=np.int64)))
        elif isinstance(sample, float):
            cols.append((k, "float64", np.array(vals, dtype=np.float64)))
        else:
            cols.append((k, "string", [str(v) for v in vals]))
    _write_parquet_pure_python(path, cols, num_rows=len(episodes))


def _hf_feature(ft: dict[str, Any]) -> dict[str, Any]:
    if ft["shape"] == [1]:
        return {"dtype": ft["dtype"], "_type": "Value"}
    return {"feature": {"dtype": ft["dtype"], "_type": "Value"}, "length": ft["shape"][0], "_type": "List"}


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=4), encoding="utf-8")
    tmp.replace(path)


# --------------------------------------------------------------------------- collector


@dataclass
class _ActiveEpisode:
    task: str
    tmp_dir: Path
    timestamps: list[float] = field(default_factory=list)
    states: list[list[float]] = field(default_factory=list)
    actions: list[list[float]] = field(default_factory=list)
    stat_images: dict[str, list[np.ndarray]] = field(default_factory=dict)
    video_writers: dict[str, _VideoWriter] = field(default_factory=dict)
    video_info: dict[str, dict[str, Any]] = field(default_factory=dict)
    camera_shapes: dict[str, tuple[int, int, int]] = field(default_factory=dict)
    prev_ts: float | None = None
    poisoned_error: str | None = None


class LeRobotSimCollector:
    """Offline LeRobot v3.0 dataset collector for headless ``SimEnv`` trajectories.

    Enforces strict fail-closed validation on ``profile_id``, ``unit_mode``, exact
    ``state``/``action`` dimensionality & finiteness, required camera frame presence/shape/finiteness,
    and strictly monotonically increasing simulation timestamps (``ts > prev_ts``).
    Failed episodes are poisoned and never committed to disk.
    """

    def __init__(
        self,
        root: str | Path,
        *,
        fps: int = 50,
        profile_id: str | None = None,
        unit_mode: str | None = None,
        robot_type: str = "rebot_b601_dm",
        joint_names: Sequence[str] | None = None,
        required_camera_roles: Sequence[str] = (),
        camera_shapes: Mapping[str, tuple[int, int, int]] | None = None,
        arm_dof: int | None = None,
        gripper_joint: int | None = None,
        task: str = "",
        video: VideoSettings | None = None,
        env: SimEnv | None = None,
    ) -> None:
        self.root = Path(root)
        if self.root.exists() and any(self.root.iterdir()):
            raise CollectorError(f"Dataset root {self.root} exists and is not empty")

        if not isinstance(profile_id, str) or not profile_id.strip():
            raise CollectorError("profile_id is required and must be a non-empty string")
        if not isinstance(unit_mode, str) or unit_mode not in VALID_UNIT_MODES:
            raise CollectorError(
                f"unit_mode is required and must be one of {VALID_UNIT_MODES}, got {unit_mode!r}"
            )
        if not isinstance(robot_type, str) or not robot_type.strip():
            raise CollectorError("robot_type must be a non-empty string")

        if env is not None and fps is None:
            fps = max(1, round(1.0 / env.control_dt))
        if _is_bool(fps) or not isinstance(fps, int) or fps <= 0:
            raise CollectorError(f"fps must be a positive integer, got {fps!r}")

        if joint_names is None:
            if env is not None:
                joint_names = env.joint_names
            else:
                raise CollectorError("joint_names is required when env is not provided")
        resolved_joints = [str(j) for j in joint_names]
        if not resolved_joints or any(not j.strip() for j in resolved_joints):
            raise CollectorError("joint_names must be a non-empty sequence of non-empty strings")
        if len(set(resolved_joints)) != len(resolved_joints):
            raise CollectorError(f"joint_names must be unique, got {resolved_joints!r}")

        roles: list[str] = []
        for role in required_camera_roles:
            if not isinstance(role, str) or not role.strip():
                raise CollectorError(f"Camera role must be a non-empty string, got {role!r}")
            if role in roles:
                raise CollectorError(f"Duplicate camera role {role!r}")
            roles.append(role)

        validated_shapes: dict[str, tuple[int, int, int]] = {}
        if camera_shapes is not None:
            for role, shp in camera_shapes.items():
                if not isinstance(role, str) or not role.strip():
                    raise CollectorError(f"Invalid camera role {role!r} in camera_shapes")
                if not isinstance(shp, (tuple, list)) or len(shp) != 3:
                    raise CollectorError(f"camera_shapes[{role!r}] must be (H, W, 3), got {shp!r}")
                h, w, c = int(shp[0]), int(shp[1]), int(shp[2])
                if h <= 0 or w <= 0 or c != 3:
                    raise CollectorError(f"camera_shapes[{role!r}] must have H,W > 0 and C == 3, got {shp!r}")
                validated_shapes[role] = (h, w, c)
        elif env is not None and env.render_camera in roles and env.renderer is not None:
            validated_shapes[env.render_camera] = (env.render_height, env.render_width, 3)

        self.fps = fps
        self.profile_id = profile_id.strip()
        self.unit_mode = unit_mode
        self.robot_type = robot_type.strip()
        self.joint_names = resolved_joints
        self.dof = len(resolved_joints)
        self.required_camera_roles = tuple(roles)
        self._camera_shapes: dict[str, tuple[int, int, int]] = validated_shapes
        self.default_task = str(task)
        self.arm_dof = arm_dof
        self.gripper_joint = gripper_joint

        self.video = video or VideoSettings()
        if self.required_camera_roles:
            self.video.vcodec = pick_vcodec(self.video.vcodec)

        self._episodes: list[dict[str, Any]] = []
        self._stats: list[dict[str, dict[str, np.ndarray]]] = []
        self._tasks: dict[str, int] = {}
        self._video_info: dict[str, dict[str, Any]] = {}
        self._ep: _ActiveEpisode | None = None

    @staticmethod
    def camera_key(role: str) -> str:
        return f"observation.images.{role}"

    def start_episode(self, task: str | None = None) -> None:
        """Begin a new episode staging buffer."""
        if self._ep is not None:
            raise CollectorError("An episode is already active; call save_episode() or discard_episode() first")
        ep_task = self.default_task if task is None else str(task)
        tmp_dir = Path(tempfile.mkdtemp(prefix="rebot-sim-ep-"))
        self._ep = _ActiveEpisode(
            task=ep_task,
            tmp_dir=tmp_dir,
            stat_images={role: [] for role in self.required_camera_roles},
            camera_shapes=dict(self._camera_shapes),
        )

    def record_step(
        self,
        timestamp: float,
        state: Sequence[float] | np.ndarray,
        action: Sequence[float] | np.ndarray,
        cameras: Mapping[str, np.ndarray] | None = None,
    ) -> int:
        """Validate and append one step to the active episode. Returns 0-based frame index."""
        ep = self._ep
        if ep is None:
            raise CollectorError("No active episode; call start_episode() first")
        if ep.poisoned_error is not None:
            raise CollectorError(
                f"Active episode was invalidated by an earlier error ({ep.poisoned_error}); discard_episode() required"
            )

        try:
            if _is_bool(timestamp) or not isinstance(timestamp, (int, float, np.integer, np.floating)):
                raise CollectorError(f"timestamp must be a finite number, got {timestamp!r}")
            ts = float(timestamp)
            if not math.isfinite(ts):
                raise CollectorError(f"timestamp must be finite, got {timestamp!r}")
            if ep.prev_ts is not None and ts <= ep.prev_ts:
                raise CollectorError(
                    f"Non-monotonic timestamp: {ts} <= previous {ep.prev_ts}"
                )

            state_vec = _validate_finite_1d(state, self.dof, "observation.state")
            action_vec = _validate_finite_1d(action, self.dof, "action")

            validated_cams: dict[str, np.ndarray] = {}
            new_shapes: dict[str, tuple[int, int, int]] = {}
            if self.required_camera_roles:
                if cameras is None or not isinstance(cameras, Mapping):
                    raise CollectorError(
                        f"cameras mapping required for roles {self.required_camera_roles}"
                    )
                for role in self.required_camera_roles:
                    if role not in cameras:
                        raise CollectorError(f"Missing required camera role {role!r}")
                    expected = ep.camera_shapes.get(role)
                    img = _validate_image(cameras[role], role, expected)
                    validated_cams[role] = img
                    if expected is None:
                        new_shapes[role] = img.shape

            # All validation passed before mutating active episode or opening encoders.
            for role, shp in new_shapes.items():
                ep.camera_shapes[role] = shp

            for role, img in validated_cams.items():
                writer = ep.video_writers.get(role)
                if writer is None:
                    h, w, _ = ep.camera_shapes[role]
                    vid_path = ep.tmp_dir / f"{role}.mp4"
                    writer = _VideoWriter(vid_path, self.fps, self.video, (w, h))
                    ep.video_writers[role] = writer
                writer.put(img)
                every = max(1, round(self.fps * STATS_IMAGE_EVERY_S))
                idx = len(ep.states)
                if idx < 50 or idx % every == 0:
                    ep.stat_images[role].append(_stats_image(img))

            frame_idx = len(ep.states)
            ep.timestamps.append(ts)
            ep.states.append(state_vec)
            ep.actions.append(action_vec)
            ep.prev_ts = ts
            return frame_idx
        except Exception as exc:
            ep.poisoned_error = str(exc)
            raise

    def discard_episode(self) -> None:
        """Abort and clean up the active episode without writing to ``root``."""
        ep = self._ep
        if ep is None:
            return
        for writer in ep.video_writers.values():
            try:
                writer.close()
            except Exception:
                pass
        shutil.rmtree(ep.tmp_dir, ignore_errors=True)
        self._ep = None

    def save_episode(self) -> int:
        """Commit the active valid episode to ``root`` in LeRobot v3.0 format."""
        ep = self._ep
        if ep is None:
            raise CollectorError("No active episode to save")
        if ep.poisoned_error is not None:
            raise CollectorError(
                f"Cannot save invalidated episode ({ep.poisoned_error}); call discard_episode()"
            )
        n = len(ep.states)
        if n == 0:
            raise CollectorError("Cannot save empty episode; record at least one step or discard_episode()")

        for role, writer in ep.video_writers.items():
            writer.close()
            info = probe_video(writer.path)
            if info["frames"] and info["frames"] != n:
                raise CollectorError(f"Camera {role!r} video has {info['frames']} frames for {n} steps")
            ep.video_info[role] = info

        idx = len(self._episodes)
        chunk, file_idx = divmod(idx, CHUNKS_SIZE)
        start = self._episodes[-1]["dataset_to_index"] if self._episodes else 0

        next_tasks = dict(self._tasks)
        task_index = next_tasks.setdefault(ep.task, len(next_tasks))

        frame_index = np.arange(n, dtype=np.int64)
        columns: dict[str, np.ndarray] = {
            "observation.state": np.asarray(ep.states, dtype=np.float32),
            "action": np.asarray(ep.actions, dtype=np.float32),
            "timestamp": np.asarray(ep.timestamps, dtype=np.float32),
            "frame_index": frame_index,
            "episode_index": np.full(n, idx, dtype=np.int64),
            "index": np.arange(start, start + n, dtype=np.int64),
            "task_index": np.full(n, task_index, dtype=np.int64),
        }

        next_camera_shapes = dict(self._camera_shapes)
        next_camera_shapes.update(ep.camera_shapes)
        next_video_info = dict(self._video_info)
        for role, vinfo in ep.video_info.items():
            next_video_info.setdefault(role, vinfo)

        features = self._build_features(next_camera_shapes, next_video_info)
        hf_meta = {
            "info": {
                "features": {k: _hf_feature(features[k]) for k in columns},
                "unit_mode": self.unit_mode,
                "profile_id": self.profile_id,
                "arm_dof": self.arm_dof,
                "gripper_joint": self.gripper_joint,
            }
        }

        self.root.mkdir(parents=True, exist_ok=True)
        data_path = self.root / DATA_PATH.format(chunk_index=chunk, file_index=file_idx)
        _write_data_parquet(data_path, columns, hf_meta)

        stats: dict[str, dict[str, np.ndarray]] = {k: feature_stats(v) for k, v in columns.items()}
        ep_row: dict[str, Any] = {
            "episode_index": idx,
            "tasks": [ep.task],
            "length": n,
            "data/chunk_index": chunk,
            "data/file_index": file_idx,
            "dataset_from_index": start,
            "dataset_to_index": start + n,
        }

        for role, writer in ep.video_writers.items():
            cam_key = self.camera_key(role)
            video_path = self.root / VIDEO_PATH.format(
                video_key=cam_key,
                chunk_index=chunk,
                file_index=file_idx,
            )
            video_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(writer.path), video_path)
            stats[cam_key] = image_stats(ep.stat_images[role])
            ep_row[f"videos/{cam_key}/chunk_index"] = chunk
            ep_row[f"videos/{cam_key}/file_index"] = file_idx
            ep_row[f"videos/{cam_key}/from_timestamp"] = 0.0
            ep_row[f"videos/{cam_key}/to_timestamp"] = n / self.fps

        for key in features:
            for stat_name, stat_val in stats[key].items():
                arr = np.asarray(stat_val)
                ep_row[f"stats/{key}/{stat_name}"] = (
                    arr.astype(np.int64).tolist() if stat_name == "count" else arr.astype(float).tolist()
                )
        ep_row["meta/episodes/chunk_index"] = 0
        ep_row["meta/episodes/file_index"] = 0

        self._camera_shapes = next_camera_shapes
        self._video_info = next_video_info
        self._tasks = next_tasks
        self._episodes.append(ep_row)
        self._stats.append(stats)

        self._write_meta(features)
        shutil.rmtree(ep.tmp_dir, ignore_errors=True)
        self._ep = None
        return idx

    def collect_episode(
        self,
        env: SimEnv,
        policy: Callable[[dict[str, Any], int], Sequence[float] | np.ndarray]
        | Sequence[Sequence[float] | np.ndarray],
        *,
        max_steps: int = 200,
        task: str | None = None,
        seed: int | None = None,
        camera_role: str | None = None,
        stop_on_done: bool = True,
    ) -> int:
        """Reset ``env``, run ``policy`` for up to ``max_steps``, and save the episode."""
        if max_steps <= 0:
            raise CollectorError(f"max_steps must be positive, got {max_steps}")

        obs, _info = env.reset(seed=seed)
        role = camera_role or (self.required_camera_roles[0] if len(self.required_camera_roles) == 1 else env.render_camera)
        self.start_episode(task=task)
        try:
            for step_idx in range(max_steps):
                act = policy(obs, step_idx) if callable(policy) else policy[step_idx]
                next_obs, _reward, done, next_info = env.step(act)
                cams = None
                if self.required_camera_roles:
                    cams = {}
                    if role is not None and "image" in obs:
                        cams[role] = obs["image"]
                self.record_step(
                    timestamp=next_info["sim_time_s"],
                    state=obs["state"],
                    action=act,
                    cameras=cams,
                )
                obs = next_obs
                if stop_on_done and done:
                    break
            return self.save_episode()
        except Exception:
            self.discard_episode()
            raise

    def _build_features(
        self,
        camera_shapes: Mapping[str, tuple[int, int, int]],
        video_info: Mapping[str, dict[str, Any]],
    ) -> dict[str, dict[str, Any]]:
        feats: dict[str, dict[str, Any]] = {
            "observation.state": {"dtype": "float32", "shape": [self.dof], "names": list(self.joint_names)},
            "action": {"dtype": "float32", "shape": [self.dof], "names": list(self.joint_names)},
        }
        for role in self.required_camera_roles:
            cam_key = self.camera_key(role)
            vi = video_info.get(role)
            shp = camera_shapes[role]
            h, w, c = shp
            info_block: dict[str, Any] = {
                "video.height": vi["video.height"] if vi else h,
                "video.width": vi["video.width"] if vi else w,
                "video.codec": vi["video.codec"] if vi else self.video.vcodec,
                "video.pix_fmt": vi["video.pix_fmt"] if vi else self.video.pix_fmt,
                "video.fps": vi["video.fps"] if vi else self.fps,
                "video.channels": c,
                "has_audio": vi["has_audio"] if vi else False,
                "video.g": self.video.g,
                "video.crf": self.video.crf,
                "video.preset": self.video.options().get("preset"),
                "video.fast_decode": 0,
                "video.video_backend": "pyav",
                "video.extra_options": {},
                "is_depth_map": False,
            }
            feats[cam_key] = {
                "dtype": "video",
                "shape": [h, w, c],
                "names": ["height", "width", "channels"],
                "info": info_block,
            }
        feats.update(json.loads(json.dumps(DEFAULT_FEATURES)))
        return feats

    def _write_meta(self, features: dict[str, dict[str, Any]]) -> None:
        meta = self.root / "meta"
        (meta / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)
        total_frames = int(self._episodes[-1]["dataset_to_index"])
        info: dict[str, Any] = {
            "codebase_version": CODEBASE_VERSION,
            "fps": self.fps,
            "features": features,
            "total_episodes": len(self._episodes),
            "total_frames": total_frames,
            "total_tasks": len(self._tasks),
            "chunks_size": CHUNKS_SIZE,
            "data_files_size_in_mb": 100,
            "video_files_size_in_mb": 200,
            "data_path": DATA_PATH,
            "video_path": VIDEO_PATH if self.required_camera_roles else None,
            "robot_type": self.robot_type,
            "splits": {"train": f"0:{len(self._episodes)}"},
            "unit_mode": self.unit_mode,
            "rebot_unit_mode": self.unit_mode,
            "profile_id": self.profile_id,
            "arm_dof": self.arm_dof,
            "gripper_joint": self.gripper_joint,
        }
        _write_json(meta / "info.json", info)
        _write_json(meta / "stats.json", _jsonable(aggregate_stats(self._stats)))
        _write_episodes_parquet(self.root / EPISODES_PATH, self._episodes)
        _write_tasks_parquet(meta / "tasks.parquet", self._tasks)

    def close(self) -> None:
        self.discard_episode()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()
