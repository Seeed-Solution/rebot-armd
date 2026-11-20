//! `tract-onnx` inference backend (gated by `feature = "policy-tract"`).

use std::io::Cursor;

use tract_onnx::prelude::*;
use tract_onnx::tract_hir::infer::Factoid;

use super::manifest::reject_external_onnx_data;
use super::{PolicyBackend, PolicyError};

pub struct TractBackend {
    plan: TypedRunnableModel<TypedModel>,
    input_dim: usize,
    output_dim: usize,
}

impl TractBackend {
    /// Load ONNX model bytes into a static-shape `[1, input_dim] -> [1, output_dim]` runnable plan.
    pub fn from_onnx_bytes(model_bytes: &[u8]) -> Result<Self, PolicyError> {
        reject_external_onnx_data(model_bytes)?;

        let mut cursor = Cursor::new(model_bytes);
        let model = tract_onnx::onnx()
            .model_for_read(&mut cursor)
            .map_err(|e| PolicyError::BackendLoad(format!("failed to parse ONNX model: {e}")))?;

        let num_inputs = model
            .inputs
            .len();
        if num_inputs != 1 {
            return Err(PolicyError::BackendLoad(format!(
                "expected single-input ONNX model, got {num_inputs} inputs"
            )));
        }
        let num_outputs = model.outputs.len();
        if num_outputs != 1 {
            return Err(PolicyError::BackendLoad(format!(
                "expected single-output ONNX model, got {num_outputs} outputs"
            )));
        }

        let inf_in = model
            .input_fact(0)
            .map_err(|e| PolicyError::BackendLoad(format!("failed to read input fact: {e}")))?;
        let in_dt = inf_in.datum_type.concretize().ok_or_else(|| {
            PolicyError::BackendLoad("ONNX input datum_type is dynamic or unspecified".into())
        })?;
        if in_dt != DatumType::F32 {
            return Err(PolicyError::BackendLoad(format!(
                "ONNX input datum_type must be f32, got {in_dt:?}"
            )));
        }
        let inf_in_shape = inf_in.shape.concretize().ok_or_else(|| {
            PolicyError::BackendLoad("ONNX input shape has dynamic rank".into())
        })?;
        let input_dim = validate_static_2d_shape(&inf_in_shape, "ONNX input")?;

        let typed = model
            .into_optimized()
            .map_err(|e| PolicyError::BackendLoad(format!("failed to optimize ONNX model: {e}")))?;

        let typed_in = typed
            .input_fact(0)
            .map_err(|e| PolicyError::BackendLoad(format!("failed to inspect typed input: {e}")))?;
        if typed_in.datum_type != DatumType::F32 {
            return Err(PolicyError::BackendLoad(format!(
                "optimized ONNX input datum_type must be f32, got {:?}",
                typed_in.datum_type
            )));
        }
        let typed_in_shape = typed_in.shape.as_concrete().ok_or_else(|| {
            PolicyError::BackendLoad("ONNX input shape contains symbolic dimensions".into())
        })?;
        if typed_in_shape != [1, input_dim] {
            return Err(PolicyError::BackendLoad(format!(
                "ONNX input shape must be [1, {input_dim}], got {typed_in_shape:?}"
            )));
        }

        let typed_out = typed
            .output_fact(0)
            .map_err(|e| PolicyError::BackendLoad(format!("failed to inspect typed output: {e}")))?;
        if typed_out.datum_type != DatumType::F32 {
            return Err(PolicyError::BackendLoad(format!(
                "ONNX output datum_type must be f32, got {:?}",
                typed_out.datum_type
            )));
        }
        let typed_out_shape = typed_out.shape.as_concrete().ok_or_else(|| {
            PolicyError::BackendLoad("ONNX output shape contains symbolic dimensions".into())
        })?;
        if typed_out_shape.len() != 2 || typed_out_shape[0] != 1 || typed_out_shape[1] == 0 {
            return Err(PolicyError::BackendLoad(format!(
                "ONNX output shape must be [1, N] with N > 0, got {typed_out_shape:?}"
            )));
        }
        let output_dim = typed_out_shape[1];

        let plan = typed
            .into_runnable()
            .map_err(|e| PolicyError::BackendLoad(format!("failed to build runnable plan: {e}")))?;

        Ok(Self {
            plan,
            input_dim,
            output_dim,
        })
    }
}

fn validate_static_2d_shape(shape: &[TDim], label: &str) -> Result<usize, PolicyError> {
    if shape.len() != 2 {
        return Err(PolicyError::BackendLoad(format!(
            "{label} rank must be 2 ([1, D]), got rank {}",
            shape.len()
        )));
    }
    let d0 = shape[0].to_i64().map_err(|_| {
        PolicyError::BackendLoad(format!("{label} dim[0] is symbolic/dynamic: {:?}", shape[0]))
    })?;
    let d1 = shape[1].to_i64().map_err(|_| {
        PolicyError::BackendLoad(format!("{label} dim[1] is symbolic/dynamic: {:?}", shape[1]))
    })?;
    if d0 != 1 || d1 <= 0 {
        return Err(PolicyError::BackendLoad(format!(
            "{label} shape must be [1, D] with D > 0, got [{d0}, {d1}]"
        )));
    }
    Ok(d1 as usize)
}

impl PolicyBackend for TractBackend {
    fn backend_name(&self) -> &'static str {
        "tract-onnx"
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
                    "TractBackend input[{i}] is non-finite: {v}"
                )));
            }
        }

        let tensor = Tensor::from_shape(&[1, self.input_dim], input).map_err(|e| {
            PolicyError::InferenceFailed(format!("failed to build input tensor: {e}"))
        })?;
        let mut outputs = self
            .plan
            .run(tvec!(tensor.into()))
            .map_err(|e| PolicyError::InferenceFailed(format!("tract execution failed: {e}")))?;

        let out_tv = outputs.pop().ok_or_else(|| {
            PolicyError::InferenceFailed("ONNX model produced no output tensors".into())
        })?;
        if out_tv.shape() != [1, self.output_dim] {
            return Err(PolicyError::InferenceFailed(format!(
                "unexpected ONNX output shape {:?}, expected [1, {}]",
                out_tv.shape(),
                self.output_dim
            )));
        }
        let slice = out_tv.as_slice::<f32>().map_err(|e| {
            PolicyError::InferenceFailed(format!("ONNX output is not f32 slice: {e}"))
        })?;
        for (i, &v) in slice.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "TractBackend output[{i}] is non-finite: {v}"
                )));
            }
        }
        output.copy_from_slice(slice);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy_runtime::mlp::{Activation, MlpBackend, MlpLayerSpec};

    fn pb_varint(mut v: u64, out: &mut Vec<u8>) {
        loop {
            let b = (v & 0x7F) as u8;
            v >>= 7;
            if v == 0 {
                out.push(b);
                break;
            }
            out.push(b | 0x80);
        }
    }

    fn pb_int(field: u64, v: u64, out: &mut Vec<u8>) {
        pb_varint((field << 3) | 0, out);
        pb_varint(v, out);
    }

    fn pb_bytes(field: u64, data: &[u8], out: &mut Vec<u8>) {
        pb_varint((field << 3) | 2, out);
        pb_varint(data.len() as u64, out);
        out.extend_from_slice(data);
    }

    fn pb_str(field: u64, s: &str, out: &mut Vec<u8>) {
        pb_bytes(field, s.as_bytes(), out);
    }

    fn make_dim(d: i64) -> Vec<u8> {
        let mut out = Vec::new();
        pb_int(1, d as u64, &mut out);
        out
    }

    fn make_sym_dim(name: &str) -> Vec<u8> {
        let mut out = Vec::new();
        pb_str(2, name, &mut out);
        out
    }

    fn make_value_info(name: &str, dims: &[Vec<u8>]) -> Vec<u8> {
        let mut shape = Vec::new();
        for d in dims {
            pb_bytes(1, d, &mut shape);
        }
        let mut tensor_type = Vec::new();
        pb_int(1, 1, &mut tensor_type); // FLOAT = 1
        pb_bytes(2, &shape, &mut tensor_type);
        let mut type_proto = Vec::new();
        pb_bytes(1, &tensor_type, &mut type_proto);
        let mut vi = Vec::new();
        pb_str(1, name, &mut vi);
        pb_bytes(2, &type_proto, &mut vi);
        vi
    }

    fn make_f32_tensor(name: &str, dims: &[i64], data: &[f32]) -> Vec<u8> {
        let mut t = Vec::new();
        for &d in dims {
            pb_int(1, d as u64, &mut t);
        }
        pb_int(2, 1, &mut t); // FLOAT = 1
        pb_str(8, name, &mut t);
        let mut raw = Vec::with_capacity(data.len() * 4);
        for &v in data {
            raw.extend_from_slice(&v.to_le_bytes());
        }
        pb_bytes(9, &raw, &mut t);
        t
    }

    fn make_node(op_type: &str, inputs: &[&str], outputs: &[&str], name: &str) -> Vec<u8> {
        let mut n = Vec::new();
        for inp in inputs {
            pb_str(1, inp, &mut n);
        }
        for out in outputs {
            pb_str(2, out, &mut n);
        }
        pb_str(3, name, &mut n);
        pb_str(4, op_type, &mut n);
        n
    }

    fn transpose_row_major(w_out_in: &[f32], out_dim: usize, in_dim: usize) -> Vec<f32> {
        let mut w_t = vec![0.0f32; in_dim * out_dim];
        for o in 0..out_dim {
            for i in 0..in_dim {
                w_t[i * out_dim + o] = w_out_in[o * in_dim + i];
            }
        }
        w_t
    }

    fn build_two_layer_mlp_onnx(l1: &MlpLayerSpec, l2: &MlpLayerSpec) -> Vec<u8> {
        let w1_t = transpose_row_major(&l1.weights, l1.out_dim, l1.in_dim);
        let w2_t = transpose_row_major(&l2.weights, l2.out_dim, l2.in_dim);

        let t_w1 = make_f32_tensor("W1", &[l1.in_dim as i64, l1.out_dim as i64], &w1_t);
        let t_b1 = make_f32_tensor("B1", &[l1.out_dim as i64], &l1.bias);
        let t_w2 = make_f32_tensor("W2", &[l2.in_dim as i64, l2.out_dim as i64], &w2_t);
        let t_b2 = make_f32_tensor("B2", &[l2.out_dim as i64], &l2.bias);

        let n1 = make_node("MatMul", &["obs", "W1"], &["mm1"], "mm1");
        let n2 = make_node("Add", &["mm1", "B1"], &["add1"], "add1");
        let n3 = make_node("Relu", &["add1"], &["h1"], "relu1");
        let n4 = make_node("MatMul", &["h1", "W2"], &["mm2"], "mm2");
        let n5 = make_node("Add", &["mm2", "B2"], &["add2"], "add2");
        let n6 = make_node("Tanh", &["add2"], &["act"], "tanh2");

        let vi_in = make_value_info("obs", &[make_dim(1), make_dim(l1.in_dim as i64)]);
        let vi_out = make_value_info("act", &[make_dim(1), make_dim(l2.out_dim as i64)]);

        let mut graph = Vec::new();
        for node in [n1, n2, n3, n4, n5, n6] {
            pb_bytes(1, &node, &mut graph);
        }
        pb_str(2, "two_layer_mlp", &mut graph);
        for init in [t_w1, t_b1, t_w2, t_b2] {
            pb_bytes(5, &init, &mut graph);
        }
        pb_bytes(11, &vi_in, &mut graph);
        pb_bytes(12, &vi_out, &mut graph);

        let mut opset = Vec::new();
        pb_str(1, "", &mut opset);
        pb_int(2, 13, &mut opset);

        let mut model = Vec::new();
        pb_int(1, 8, &mut model);
        pb_str(2, "armd-test", &mut model);
        pb_bytes(7, &graph, &mut model);
        pb_bytes(8, &opset, &mut model);
        model
    }

    #[test]
    fn tract_backend_matches_mlp_backend_within_1e5() {
        let l1 = MlpLayerSpec::new(
            6,
            8,
            Activation::Relu,
            (0..48).map(|i| ((i as f32) * 0.13 - 2.5) * 0.1).collect(),
            vec![0.1, -0.2, 0.05, 0.3, -0.15, 0.25, -0.05, 0.12],
        );
        let l2 = MlpLayerSpec::new(
            8,
            6,
            Activation::Tanh,
            (0..48).map(|i| ((i as f32) * 0.19 - 3.1) * 0.08).collect(),
            vec![0.02, -0.04, 0.06, -0.01, 0.03, -0.05],
        );

        let onnx_bytes = build_two_layer_mlp_onnx(&l1, &l2);
        let mut mlp = MlpBackend::from_layers(vec![l1, l2]).unwrap();
        let mut tract = TractBackend::from_onnx_bytes(&onnx_bytes).unwrap();

        assert_eq!(tract.backend_name(), "tract-onnx");
        assert_eq!(tract.input_dim(), 6);
        assert_eq!(tract.output_dim(), 6);

        let input = [0.4f32, -1.1, 0.75, 0.0, -0.3, 1.25];
        let mut mlp_out = [0.0f32; 6];
        let mut tract_out = [0.0f32; 6];
        mlp.infer(&input, &mut mlp_out).unwrap();
        tract.infer(&input, &mut tract_out).unwrap();

        for i in 0..6 {
            assert!(
                (mlp_out[i] - tract_out[i]).abs() < 1e-5,
                "mismatch at {i}: mlp={} vs tract={}",
                mlp_out[i],
                tract_out[i]
            );
        }
    }

    #[test]
    fn tract_backend_rejects_dynamic_symbolic_shape() {
        let w = make_f32_tensor("W", &[6, 6], &[0.0; 36]);
        let n = make_node("MatMul", &["obs", "W"], &["act"], "mm");
        let vi_in = make_value_info("obs", &[make_sym_dim("batch"), make_dim(6)]);
        let vi_out = make_value_info("act", &[make_sym_dim("batch"), make_dim(6)]);

        let mut graph = Vec::new();
        pb_bytes(1, &n, &mut graph);
        pb_bytes(5, &w, &mut graph);
        pb_bytes(11, &vi_in, &mut graph);
        pb_bytes(12, &vi_out, &mut graph);

        let mut opset = Vec::new();
        pb_str(1, "", &mut opset);
        pb_int(2, 13, &mut opset);

        let mut model = Vec::new();
        pb_int(1, 8, &mut model);
        pb_bytes(7, &graph, &mut model);
        pb_bytes(8, &opset, &mut model);

        let err = TractBackend::from_onnx_bytes(&model).err().unwrap();
        assert!(matches!(err, PolicyError::BackendLoad(_)));
    }
}
