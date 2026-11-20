//! Zero-dependency, zero-allocation-at-inference MLP backend.
//!
//! Supports:
//! - Binary `RMLP\x01` format (little-endian header + sequential dense layers)
//! - JSON `.mlp.json` format (`{"layers": [{"in_dim", "out_dim", "activation", "weights", "bias"}]}`)

use std::f64::consts::FRAC_1_SQRT_2;

use super::{PolicyBackend, PolicyError};

pub const RMLP_MAGIC: &[u8; 5] = b"RMLP\x01";
pub const MAX_LAYERS: usize = 16;
pub const MAX_DIM: usize = 1024;
pub const MAX_PARAMS: usize = 2_000_000;
pub const MAX_MACS: usize = 4_000_000;
/// Largest `.mlp.json` accepted (checked before parsing). About 400k parameters as text;
/// larger models use the binary `.rmlp` format.
pub const MAX_MLP_JSON_BYTES: usize = 4 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Activation {
    None,
    Relu,
    Tanh,
    Gelu,
}

impl Activation {
    fn from_tag(tag: u8) -> Result<Self, PolicyError> {
        match tag {
            0 => Ok(Self::None),
            1 => Ok(Self::Relu),
            2 => Ok(Self::Tanh),
            3 => Ok(Self::Gelu),
            other => Err(PolicyError::InvalidArtifact(format!(
                "unknown RMLP activation tag {other}"
            ))),
        }
    }

    fn to_tag(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Relu => 1,
            Self::Tanh => 2,
            Self::Gelu => 3,
        }
    }

    fn from_str_name(name: &str) -> Result<Self, PolicyError> {
        match name.trim().to_ascii_lowercase().as_str() {
            "none" | "identity" | "linear" => Ok(Self::None),
            "relu" => Ok(Self::Relu),
            "tanh" => Ok(Self::Tanh),
            "gelu" => Ok(Self::Gelu),
            other => Err(PolicyError::InvalidArtifact(format!(
                "unknown MLP activation {other:?}; expected none, relu, tanh, or gelu"
            ))),
        }
    }

    #[inline]
    fn apply(self, x: f32) -> f32 {
        match self {
            Self::None => x,
            Self::Relu => x.max(0.0),
            Self::Tanh => x.tanh(),
            Self::Gelu => {
                let xd = x as f64;
                (0.5 * xd * (1.0 + erf_f64(xd * FRAC_1_SQRT_2))) as f32
            }
        }
    }
}

/// Abramowitz & Stegun 7.1.26 error function approximation (max error ~1.5e-7, exact to f32 precision).
#[inline]
fn erf_f64(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let ax = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * ax);
    let poly = (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t
        + 0.254829592)
        * t;
    sign * (1.0 - poly * (-ax * ax).exp())
}

/// Dense layer specification: `weights` is row-major `[out_dim, in_dim]`.
#[derive(Debug, Clone, PartialEq)]
pub struct MlpLayerSpec {
    pub in_dim: usize,
    pub out_dim: usize,
    pub activation: Activation,
    pub weights: Vec<f32>,
    pub bias: Vec<f32>,
}

impl MlpLayerSpec {
    pub fn new(
        in_dim: usize,
        out_dim: usize,
        activation: Activation,
        weights: Vec<f32>,
        bias: Vec<f32>,
    ) -> Self {
        Self {
            in_dim,
            out_dim,
            activation,
            weights,
            bias,
        }
    }
}

pub struct MlpBackend {
    layers: Vec<MlpLayerSpec>,
    input_dim: usize,
    output_dim: usize,
    scratch: [Vec<f32>; 2],
}

impl MlpBackend {
    pub fn from_layers(layers: Vec<MlpLayerSpec>) -> Result<Self, PolicyError> {
        if layers.is_empty() || layers.len() > MAX_LAYERS {
            return Err(PolicyError::InvalidArtifact(format!(
                "MLP layer count {} must be in 1..={MAX_LAYERS}",
                layers.len()
            )));
        }

        let mut total_params: usize = 0;
        let mut total_macs: usize = 0;
        let mut max_width: usize = 0;

        for (idx, layer) in layers.iter().enumerate() {
            if layer.in_dim == 0 || layer.in_dim > MAX_DIM || layer.out_dim == 0 || layer.out_dim > MAX_DIM {
                return Err(PolicyError::InvalidArtifact(format!(
                    "layer {idx} dims ({}, {}) must be in 1..={MAX_DIM}",
                    layer.in_dim, layer.out_dim
                )));
            }
            if idx > 0 && layer.in_dim != layers[idx - 1].out_dim {
                return Err(PolicyError::DimensionMismatch {
                    expected: layers[idx - 1].out_dim,
                    actual: layer.in_dim,
                });
            }
            let w_len = layer.in_dim.checked_mul(layer.out_dim).ok_or_else(|| {
                PolicyError::InvalidArtifact("MLP weight count overflow".into())
            })?;
            if layer.weights.len() != w_len {
                return Err(PolicyError::DimensionMismatch {
                    expected: w_len,
                    actual: layer.weights.len(),
                });
            }
            if layer.bias.len() != layer.out_dim {
                return Err(PolicyError::DimensionMismatch {
                    expected: layer.out_dim,
                    actual: layer.bias.len(),
                });
            }

            total_macs = total_macs
                .checked_add(w_len)
                .ok_or_else(|| PolicyError::InvalidArtifact("MLP MAC count overflow".into()))?;
            total_params = total_params
                .checked_add(w_len)
                .and_then(|p| p.checked_add(layer.out_dim))
                .ok_or_else(|| PolicyError::InvalidArtifact("MLP param count overflow".into()))?;
            if total_params > MAX_PARAMS {
                return Err(PolicyError::InvalidArtifact(format!(
                    "MLP parameter count {total_params} exceeds MAX_PARAMS ({MAX_PARAMS})"
                )));
            }
            if total_macs > MAX_MACS {
                return Err(PolicyError::InvalidArtifact(format!(
                    "MLP MAC count {total_macs} exceeds MAX_MACS ({MAX_MACS})"
                )));
            }

            for (w_idx, &w) in layer.weights.iter().enumerate() {
                if !w.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "layer {idx} weight[{w_idx}] is non-finite: {w}"
                    )));
                }
            }
            for (b_idx, &b) in layer.bias.iter().enumerate() {
                if !b.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "layer {idx} bias[{b_idx}] is non-finite: {b}"
                    )));
                }
            }

            max_width = max_width.max(layer.in_dim).max(layer.out_dim);
        }

        let input_dim = layers[0].in_dim;
        let output_dim = layers[layers.len() - 1].out_dim;

        Ok(Self {
            layers,
            input_dim,
            output_dim,
            scratch: [vec![0.0; max_width], vec![0.0; max_width]],
        })
    }

    /// Decode binary `RMLP\x01` format, enforcing hard limits BEFORE allocating layer vectors.
    pub fn from_rmlp_bytes(bytes: &[u8]) -> Result<Self, PolicyError> {
        if bytes.len() < 9 || &bytes[..5] != RMLP_MAGIC {
            return Err(PolicyError::InvalidArtifact(
                "invalid RMLP header (expected magic RMLP\\x01)".into(),
            ));
        }
        let num_layers = u32::from_le_bytes(bytes[5..9].try_into().unwrap()) as usize;
        if num_layers == 0 || num_layers > MAX_LAYERS {
            return Err(PolicyError::InvalidArtifact(format!(
                "RMLP num_layers {num_layers} must be in 1..={MAX_LAYERS}"
            )));
        }

        let mut pos = 9usize;
        let mut total_params: usize = 0;
        let mut total_macs: usize = 0;
        let mut prev_out_dim: Option<usize> = None;
        let mut layers = Vec::with_capacity(num_layers);

        for idx in 0..num_layers {
            if pos + 9 > bytes.len() {
                return Err(PolicyError::InvalidArtifact(format!(
                    "truncated RMLP layer {idx} header"
                )));
            }
            let in_dim = u32::from_le_bytes(bytes[pos..pos + 4].try_into().unwrap()) as usize;
            let out_dim = u32::from_le_bytes(bytes[pos + 4..pos + 8].try_into().unwrap()) as usize;
            let act = Activation::from_tag(bytes[pos + 8])?;
            pos += 9;

            if in_dim == 0 || in_dim > MAX_DIM || out_dim == 0 || out_dim > MAX_DIM {
                return Err(PolicyError::InvalidArtifact(format!(
                    "RMLP layer {idx} dims ({in_dim}, {out_dim}) must be in 1..={MAX_DIM}"
                )));
            }
            if let Some(prev) = prev_out_dim {
                if in_dim != prev {
                    return Err(PolicyError::DimensionMismatch {
                        expected: prev,
                        actual: in_dim,
                    });
                }
            }
            prev_out_dim = Some(out_dim);

            let w_count = in_dim.checked_mul(out_dim).ok_or_else(|| {
                PolicyError::InvalidArtifact("RMLP weight count overflow".into())
            })?;
            total_macs = total_macs
                .checked_add(w_count)
                .ok_or_else(|| PolicyError::InvalidArtifact("RMLP MAC count overflow".into()))?;
            total_params = total_params
                .checked_add(w_count)
                .and_then(|p| p.checked_add(out_dim))
                .ok_or_else(|| PolicyError::InvalidArtifact("RMLP param count overflow".into()))?;

            if total_params > MAX_PARAMS {
                return Err(PolicyError::InvalidArtifact(format!(
                    "RMLP parameter count {total_params} exceeds MAX_PARAMS ({MAX_PARAMS})"
                )));
            }
            if total_macs > MAX_MACS {
                return Err(PolicyError::InvalidArtifact(format!(
                    "RMLP MAC count {total_macs} exceeds MAX_MACS ({MAX_MACS})"
                )));
            }

            let w_bytes = w_count.checked_mul(4).ok_or_else(|| {
                PolicyError::InvalidArtifact("RMLP weight byte size overflow".into())
            })?;
            let b_bytes = out_dim.checked_mul(4).ok_or_else(|| {
                PolicyError::InvalidArtifact("RMLP bias byte size overflow".into())
            })?;
            let needed = w_bytes.checked_add(b_bytes).ok_or_else(|| {
                PolicyError::InvalidArtifact("RMLP layer byte size overflow".into())
            })?;
            if pos + needed > bytes.len() {
                return Err(PolicyError::InvalidArtifact(format!(
                    "truncated RMLP layer {idx} payload"
                )));
            }

            let mut weights = Vec::with_capacity(w_count);
            for c in bytes[pos..pos + w_bytes].chunks_exact(4) {
                let v = f32::from_le_bytes(c.try_into().unwrap());
                if !v.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "RMLP layer {idx} contains non-finite weight"
                    )));
                }
                weights.push(v);
            }
            pos += w_bytes;

            let mut bias = Vec::with_capacity(out_dim);
            for c in bytes[pos..pos + b_bytes].chunks_exact(4) {
                let v = f32::from_le_bytes(c.try_into().unwrap());
                if !v.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "RMLP layer {idx} contains non-finite bias"
                    )));
                }
                bias.push(v);
            }
            pos += b_bytes;

            layers.push(MlpLayerSpec::new(in_dim, out_dim, act, weights, bias));
        }

        if pos != bytes.len() {
            return Err(PolicyError::InvalidArtifact(format!(
                "RMLP payload has {} trailing bytes",
                bytes.len() - pos
            )));
        }

        Self::from_layers(layers)
    }

    /// Decode JSON MLP format (`{"layers": [...]}`). The file size is checked against
    /// [`MAX_MLP_JSON_BYTES`] before parsing, and the document is deserialized straight into
    /// typed vectors (no `serde_json::Value` tree), so parsing allocates O(file size).
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PolicyError> {
        #[derive(serde::Deserialize)]
        struct JsonLayer {
            in_dim: Option<u64>,
            out_dim: Option<u64>,
            activation: Option<String>,
            weights: Option<Vec<f64>>,
            bias: Option<Vec<f64>>,
        }
        #[derive(serde::Deserialize)]
        struct JsonMlp {
            format: Option<String>,
            layers: Option<Vec<JsonLayer>>,
        }
        if bytes.len() > MAX_MLP_JSON_BYTES {
            return Err(PolicyError::ResourceBudget(format!(
                "MLP JSON is {} bytes; the limit is {MAX_MLP_JSON_BYTES} bytes (use the binary .rmlp format for larger models)",
                bytes.len()
            )));
        }
        let root: JsonMlp = serde_json::from_slice(bytes)
            .map_err(|e| PolicyError::InvalidArtifact(format!("invalid MLP JSON: {e}")))?;
        if let Some(s) = root.format.as_deref() {
            if s != "rebot-mlp/v1" {
                return Err(PolicyError::InvalidArtifact(format!(
                    "unsupported MLP JSON format {s:?}"
                )));
            }
        }
        let layers_arr = root
            .layers
            .ok_or_else(|| PolicyError::InvalidArtifact("MLP JSON must contain 'layers' array".into()))?;
        if layers_arr.is_empty() || layers_arr.len() > MAX_LAYERS {
            return Err(PolicyError::InvalidArtifact(format!(
                "MLP JSON layer count {} must be in 1..={MAX_LAYERS}",
                layers_arr.len()
            )));
        }

        let mut total_params: usize = 0;
        let mut total_macs: usize = 0;
        let mut prev_out_dim: Option<usize> = None;
        let mut layers = Vec::with_capacity(layers_arr.len());

        for (idx, item) in layers_arr.into_iter().enumerate() {
            let in_dim = item
                .in_dim
                .ok_or_else(|| PolicyError::InvalidArtifact(format!("layer {idx} missing in_dim")))?
                as usize;
            let out_dim = item
                .out_dim
                .ok_or_else(|| PolicyError::InvalidArtifact(format!("layer {idx} missing out_dim")))?
                as usize;

            if in_dim == 0 || in_dim > MAX_DIM || out_dim == 0 || out_dim > MAX_DIM {
                return Err(PolicyError::InvalidArtifact(format!(
                    "layer {idx} dims ({in_dim}, {out_dim}) must be in 1..={MAX_DIM}"
                )));
            }
            if let Some(prev) = prev_out_dim {
                if in_dim != prev {
                    return Err(PolicyError::DimensionMismatch {
                        expected: prev,
                        actual: in_dim,
                    });
                }
            }
            prev_out_dim = Some(out_dim);

            let w_count = in_dim.checked_mul(out_dim).ok_or_else(|| {
                PolicyError::InvalidArtifact("MLP weight count overflow".into())
            })?;
            total_macs = total_macs
                .checked_add(w_count)
                .ok_or_else(|| PolicyError::InvalidArtifact("MLP MAC count overflow".into()))?;
            total_params = total_params
                .checked_add(w_count)
                .and_then(|p| p.checked_add(out_dim))
                .ok_or_else(|| PolicyError::InvalidArtifact("MLP param count overflow".into()))?;
            if total_params > MAX_PARAMS {
                return Err(PolicyError::InvalidArtifact(format!(
                    "MLP parameter count {total_params} exceeds MAX_PARAMS ({MAX_PARAMS})"
                )));
            }
            if total_macs > MAX_MACS {
                return Err(PolicyError::InvalidArtifact(format!(
                    "MLP MAC count {total_macs} exceeds MAX_MACS ({MAX_MACS})"
                )));
            }

            let activation = Activation::from_str_name(item.activation.as_deref().unwrap_or("none"))?;

            let to_f32 = |vals: Vec<f64>, expected: usize, what: &str| -> Result<Vec<f32>, PolicyError> {
                if vals.len() != expected {
                    return Err(PolicyError::DimensionMismatch { expected, actual: vals.len() });
                }
                vals.into_iter()
                    .enumerate()
                    .map(|(i, v)| {
                        let f = v as f32;
                        if v.is_finite() && f.is_finite() {
                            Ok(f)
                        } else {
                            Err(PolicyError::NonFinite(format!("layer {idx} {what}[{i}] is non-finite")))
                        }
                    })
                    .collect()
            };
            let weights = to_f32(
                item.weights.ok_or_else(|| PolicyError::InvalidArtifact(format!("layer {idx} missing weights")))?,
                w_count,
                "weight",
            )?;
            let bias = to_f32(
                item.bias.ok_or_else(|| PolicyError::InvalidArtifact(format!("layer {idx} missing bias")))?,
                out_dim,
                "bias",
            )?;

            layers.push(MlpLayerSpec::new(in_dim, out_dim, activation, weights, bias));
        }

        Self::from_layers(layers)
    }
}

impl PolicyBackend for MlpBackend {
    fn backend_name(&self) -> &'static str {
        "mlp"
    }

    fn input_dim(&self) -> usize {
        self.input_dim
    }

    fn output_dim(&self) -> usize {
        self.output_dim
    }

    fn infer(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
        if input.len() != self.input_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: self.input_dim,
                actual: input.len(),
            });
        }
        if output.len() != self.output_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: self.output_dim,
                actual: output.len(),
            });
        }
        for (i, &v) in input.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "MLP input[{i}] is non-finite: {v}"
                )));
            }
        }

        self.scratch[0][..self.input_dim].copy_from_slice(input);
        let mut active_idx = 0usize;

        for layer in &self.layers {
            let (left, right) = self.scratch.split_at_mut(1);
            let (src, dst): (&[f32], &mut [f32]) = if active_idx == 0 {
                (&left[0][..layer.in_dim], &mut right[0][..layer.out_dim])
            } else {
                (&right[0][..layer.in_dim], &mut left[0][..layer.out_dim])
            };

            for o in 0..layer.out_dim {
                let row = &layer.weights[o * layer.in_dim..(o + 1) * layer.in_dim];
                let mut acc = layer.bias[o];
                for i in 0..layer.in_dim {
                    acc += row[i] * src[i];
                }
                let val = layer.activation.apply(acc);
                if !val.is_finite() {
                    return Err(PolicyError::NonFinite(format!(
                        "MLP produced non-finite activation at out_idx {o}"
                    )));
                }
                dst[o] = val;
            }
            active_idx ^= 1;
        }

        let final_buf = &self.scratch[active_idx][..self.output_dim];
        output.copy_from_slice(final_buf);
        Ok(())
    }
}

/// Encode layers into the binary `RMLP\x01` format.
pub fn encode_rmlp_bytes(layers: &[MlpLayerSpec]) -> Result<Vec<u8>, PolicyError> {
    if layers.is_empty() || layers.len() > MAX_LAYERS {
        return Err(PolicyError::InvalidArtifact(format!(
            "RMLP layer count {} must be in 1..={MAX_LAYERS}",
            layers.len()
        )));
    }
    let mut out = Vec::new();
    out.extend_from_slice(RMLP_MAGIC);
    out.extend_from_slice(&(layers.len() as u32).to_le_bytes());
    for (idx, layer) in layers.iter().enumerate() {
        let expected_w = layer.in_dim.checked_mul(layer.out_dim).ok_or_else(|| {
            PolicyError::InvalidArtifact("RMLP weight size overflow".into())
        })?;
        if layer.weights.len() != expected_w || layer.bias.len() != layer.out_dim {
            return Err(PolicyError::InvalidArtifact(format!(
                "layer {idx} weights/bias lengths do not match dims ({}, {})",
                layer.in_dim, layer.out_dim
            )));
        }
        out.extend_from_slice(&(layer.in_dim as u32).to_le_bytes());
        out.extend_from_slice(&(layer.out_dim as u32).to_le_bytes());
        out.push(layer.activation.to_tag());
        for &w in &layer.weights {
            if !w.is_finite() {
                return Err(PolicyError::NonFinite("non-finite weight in encode_rmlp_bytes".into()));
            }
            out.extend_from_slice(&w.to_le_bytes());
        }
        for &b in &layer.bias {
            if !b.is_finite() {
                return Err(PolicyError::NonFinite("non-finite bias in encode_rmlp_bytes".into()));
            }
            out.extend_from_slice(&b.to_le_bytes());
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mlp_exact_numerical_output_and_activations() {
        // 2-layer MLP: 3 -> 4 (ReLU) -> 2 (Tanh)
        let l1 = MlpLayerSpec::new(
            3,
            4,
            Activation::Relu,
            vec![
                1.0, -2.0, 0.5,
                -1.0, 0.0, 2.0,
                0.25, 0.5, -0.75,
                -3.0, -1.0, -2.0,
            ],
            vec![0.1, -0.2, 0.3, 0.0],
        );
        let l2 = MlpLayerSpec::new(
            4,
            2,
            Activation::Tanh,
            vec![
                0.5, -0.5, 1.0, 2.0,
                -1.0, 0.25, 0.5, -0.5,
            ],
            vec![0.05, -0.1],
        );
        let rmlp = encode_rmlp_bytes(&[l1.clone(), l2.clone()]).unwrap();
        let mut backend = MlpBackend::from_rmlp_bytes(&rmlp).unwrap();

        let x = [1.5f32, 0.25, -0.5];
        let mut out = [0.0f32; 2];
        backend.infer(&x, &mut out).unwrap();

        // Manual reference calculation
        // h1_0 = (1.5 - 0.5 - 0.25 + 0.1) = 0.85 -> relu = 0.85
        // h1_1 = (-1.5 + 0.0 - 1.0 - 0.2) = -2.7 -> relu = 0.0
        // h1_2 = (0.375 + 0.125 + 0.375 + 0.3) = 1.175 -> relu = 1.175
        // h1_3 = (-4.5 - 0.25 + 1.0 + 0.0) = -3.75 -> relu = 0.0
        let y0 = (0.5f32 * 0.85 + 1.0 * 1.175 + 0.05).tanh();
        let y1 = (-0.85f32 + 0.5 * 1.175 - 0.1).tanh();
        assert!((out[0] - y0).abs() < 1e-6);
        assert!((out[1] - y1).abs() < 1e-6);
    }

    #[test]
    fn mlp_rejects_hard_limits_nan_and_malformed_bytes() {
        // Layer count > MAX_LAYERS
        let mut bad_layers = Vec::new();
        for _ in 0..=MAX_LAYERS {
            bad_layers.push(MlpLayerSpec::new(1, 1, Activation::None, vec![1.0], vec![0.0]));
        }
        assert!(MlpBackend::from_layers(bad_layers).is_err());

        // Dim > MAX_DIM in binary header (must fail before allocation)
        let mut oversized_hdr = Vec::from(*RMLP_MAGIC);
        oversized_hdr.extend_from_slice(&1u32.to_le_bytes());
        oversized_hdr.extend_from_slice(&((MAX_DIM + 1) as u32).to_le_bytes());
        oversized_hdr.extend_from_slice(&1u32.to_le_bytes());
        oversized_hdr.push(0);
        assert!(MlpBackend::from_rmlp_bytes(&oversized_hdr).is_err());

        // Exceed MAX_PARAMS across multiple 1024x1024 layers
        let mut too_many_params = Vec::from(*RMLP_MAGIC);
        too_many_params.extend_from_slice(&2u32.to_le_bytes());
        for _ in 0..2 {
            too_many_params.extend_from_slice(&(1024u32).to_le_bytes());
            too_many_params.extend_from_slice(&(1024u32).to_le_bytes());
            too_many_params.push(0);
        }
        assert!(MlpBackend::from_rmlp_bytes(&too_many_params).is_err());

        // NaN weight rejection
        let nan_layer = MlpLayerSpec::new(2, 2, Activation::None, vec![1.0, f32::NAN, 0.0, 1.0], vec![0.0, 0.0]);
        assert!(matches!(
            MlpBackend::from_layers(vec![nan_layer]),
            Err(PolicyError::NonFinite(_))
        ));

        // Malformed bytes (truncated, bad magic, unknown activation, trailing garbage) -> zero panic
        for garbage in [
            b"".as_slice(),
            b"NOPE",
            b"RMLP\x01\x01\x00\x00\x00",
            b"RMLP\x01\x01\x00\x00\x00\x02\x00\x00\x00\x02\x00\x00\x00\x7f",
        ] {
            assert!(MlpBackend::from_rmlp_bytes(garbage).is_err());
        }
    }
}
