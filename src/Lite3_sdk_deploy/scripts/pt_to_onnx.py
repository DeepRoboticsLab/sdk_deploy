import torch
import onnx
import onnxruntime
import numpy as np

# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------
MODEL_PATH = "/path/to/your/policy/example.pt"   # <-- Change this
ONNX_PATH = "m20_platform4.onnx"

# Example input (replace with your real shape if different)
# Here: batch_size=1, 53*6 = 318 features
state = torch.randn(1, 53 * 6, dtype=torch.float32)

# ------------------------------------------------------------------
# 1. Load the TorchScript model
# ------------------------------------------------------------------
print("Loading TorchScript model...")
model = torch.jit.load(MODEL_PATH)
model.eval()  # Important: switch to evaluation mode

# Test forward pass (optional, just to verify loading)
with torch.no_grad():
    torch_output = model(state)
    print(f"Torch forward output shape: {torch_output.shape}")

# ------------------------------------------------------------------
# 2. Export to ONNX
# ------------------------------------------------------------------
print(f"Exporting model to ONNX -> {ONNX_PATH}")
torch.onnx.export(
    model,                     # model
    state,                     # model input (tuple if multiple)
    ONNX_PATH,                 # output file
    export_params=True,        # store trained weights
    opset_version=17,          # recent stable opset
    do_constant_folding=True,  # optimize constants
    input_names=["state"],     # name of input node
    output_names=["output"],   # name of output node
    dynamic_axes={             # optional: support variable batch size
        "state": {0: "batch"},
        "output": {0: "batch"},
    },
)

print("Export successful!")

# ------------------------------------------------------------------
# 3. Validate the exported ONNX model
# ------------------------------------------------------------------
print("Checking ONNX model integrity...")
onnx_model = onnx.load(ONNX_PATH)
onnx.checker.check_model(onnx_model)
print("ONNX model is valid")

# Print output info (name, shape, type)
for output in onnx_model.graph.output:
    print(f"Output: {output.name}")
    for dim in output.type.tensor_type.shape.dim:
        print(f"    dim {dim.dim_param or dim.dim_value}")

# ------------------------------------------------------------------
# 4. Run inference with ONNX Runtime
# ------------------------------------------------------------------
print("Creating ONNX Runtime session...")
ort_session = onnxruntime.InferenceSession(ONNX_PATH)

# Prepare input dict
ort_inputs = {"state": state.numpy()}

# Run inference
ort_outputs = ort_session.run(None, ort_inputs)  # None = return all outputs

print(f"ONNX Runtime output shape: {ort_outputs[0].shape}")
print(f"ONNX Runtime output (first 5 elements): {ort_outputs[0].flat[:5]}")

# ------------------------------------------------------------------
# 5. Compare Torch vs ONNX outputs (sanity check)
# ------------------------------------------------------------------
torch_out_np = torch_output.numpy()
onnx_out_np = ort_outputs[0]

max_diff = np.abs(torch_out_np - onnx_out_np).max()
print(f"Max difference between Torch and ONNX: {max_diff:.2e}")

if max_diff < 1e-5:
    print("Torch and ONNX outputs match perfectly!")
else:
    print("Outputs differ slightly (normal for float32 precision)")