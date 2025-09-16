import argparse
import tarfile
import json
import onnxruntime as ort
import numpy as np
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kinfer_path", type=str, help="Path to saved model file")
    args = parser.parse_args()

    print("Loading kinfer model from", args.kinfer_path)
    if not args.kinfer_path or not args.kinfer_path.endswith('.kinfer'): # .tar.gz really
        raise ValueError("Model path must be provided and end with .kinfer")

    with tarfile.open(args.kinfer_path, "r:gz") as tar:
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

    # Create dummy inputs for step function
    step_dummy_inputs = {}
    for inp in step_inputs:
        step_dummy_inputs[inp.name] = np.zeros(inp.shape, dtype=np.float32)

    print("\n=== Testing Initialization ===")
    # Test initialization
    print("Running initialization...")
    init_start = time.perf_counter()
    init_outputs = init_session.run(None, {})
    init_end = time.perf_counter()
    init_time = (init_end - init_start) * 1000
    print(f"Initialization completed in {init_time:.2f}ms")
    print(f"Init outputs shapes: {[out.shape for out in init_outputs]}")

    print("\n=== Benchmarking Step Forward Passes ===")
    # Warmup step function
    print("Warming up step function...")
    for _ in range(10):
        step_session.run(None, step_dummy_inputs)

    # Benchmark step function
    num_passes = 1000
    times = []
    print(f"\nBenchmarking {num_passes} step forward passes...")

    for i in range(num_passes):
        start = time.perf_counter()
        step_outputs = step_session.run(None, step_dummy_inputs)
        end = time.perf_counter()
        dt = (end - start) * 1000  # Convert to ms
        times.append(dt)
        print(f"Step {i+1}: {dt:.2f}ms")

    # Print statistics
    avg_time = sum(times) / len(times)
    min_time = min(times)
    max_time = max(times)

    print("\nStep function benchmark results:")
    print(f"avg: {avg_time:.2f}ms")
    print(f"min: {min_time:.2f}ms")
    print(f"max: {max_time:.2f}ms")

    print(f"\nStep outputs shapes: {[out.shape for out in step_outputs]}")


if __name__ == "__main__":
    main()
