
import tarfile
import json
import onnxruntime as ort
import numpy as np


def get_onnx_sessions(kinfer_path):
    print("Loading kinfer model from", kinfer_path)
    if not kinfer_path or not kinfer_path.endswith('.kinfer'): # .tar.gz really
        raise ValueError("Model path must be provided and end with .kinfer")

    with tarfile.open(kinfer_path, "r:gz") as tar:
        assert tar.getnames() == ["init_fn.onnx", "step_fn.onnx", "metadata.json"]

        init_model_bytes = tar.extractfile("init_fn.onnx").read()
        step_model_bytes = tar.extractfile("step_fn.onnx").read()
        metadata = json.load(tar.extractfile("metadata.json"))
        print("kinfer model metadata:", metadata)

    print("Creating ONNX inference sessions...")
    init_session = ort.InferenceSession(init_model_bytes)
    step_session = ort.InferenceSession(step_model_bytes)

    init_inputs = init_session.get_inputs()
    init_outputs = init_session.get_outputs()
    step_inputs = step_session.get_inputs()
    step_outputs = step_session.get_outputs()

    print(f"\nInit function - Inputs: {[inp.name for inp in init_inputs]}, Outputs: {[out.name for out in init_outputs]}")
    print(f"Step function - Inputs: {[inp.name for inp in step_inputs]}, Outputs: {[out.name for out in step_outputs]}")

    # warm up step function
    step_dummy_inputs = {}
    for inp in step_inputs:
        step_dummy_inputs[inp.name] = np.zeros(inp.shape, dtype=np.float32)
    for _ in range(100):
        step_session.run(None, step_dummy_inputs)

    return init_session, step_session, metadata

