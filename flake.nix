{
  description = "Devshell and derivation for firmware";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
        rustPlatform = pkgs.makeRustPlatform {
          cargo = pkgs.rust-bin.nightly.latest.minimal;
          rustc = pkgs.rust-bin.nightly.latest.minimal;
        };
      in {
        devShell = pkgs.mkShell {
          buildInputs = with pkgs; [
            # Necessary libraries
            openssl
            pkg-config

            # A rust toolchain locked to a new enough version for type_alias_impl_trait
            (pkgs.rust-bin.nightly.latest.default.override {
              extensions = [ "rust-analyzer" "rust-src" "clippy" ];
            })
          ];

          # Allow rust to find openssl
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [ pkgs.openssl ];
        };

        packages = rec {
          default = feaux-rtos;
          feaux-rtos = let
            packageConfig =
              (builtins.fromTOML (builtins.readFile ./Cargo.toml)).package;

            # Download ONNX Runtime for ort-sys
            onnxruntimeVersion = "1.22.0";
            runtimeUrl = (system:
              "https://github.com/microsoft/onnxruntime/releases/download/v${onnxruntimeVersion}/onnxruntime-${system}-${onnxruntimeVersion}.tgz");
            onnxruntimeUrls = {
              x86_64-linux = runtimeUrl "linux-x64";
              aarch64-linux = runtimeUrl "linux-aarch64";
              x86_64-darwin = runtimeUrl "osx-x86_64";
              aarch64-darwin = runtimeUrl "osx-arm64";
            };
            onnxruntimeHash = {
              x86_64-linux =
                "1cs9pql6512ilh8x7ddgm8wsz787c87vab9lrqhm1g6mjdgxai43";
              aarch64-linux =
                "00j7w93y74wi4qpnshgh7c7hzn745y7npp4jf0nbal6ij983jxmv";
              x86_64-darwin =
                "1w0psv4idpng49h1iykgmxc9jjafm6d5cii8n6qlzrvdd6kr9v74";
              aarch64-darwin =
                "01ycl70h9cpmfk5wf5xhg4ryqpyl764kmdz7j19pgv77fyyxrdna";
            };

            onnxruntime = pkgs.fetchurl {
              url = onnxruntimeUrls.${system};
              sha256 = onnxruntimeHash.${system};
            };

            # Extract ONNX Runtime
            onnxruntimeExtracted =
              pkgs.runCommand "onnxruntime-extracted" { } ''
                mkdir -p $out
                tar -xzf ${onnxruntime} -C $out --strip-components=1
              '';

          in rustPlatform.buildRustPackage {
            pname = "faux-rtos";

            version = packageConfig.version;
            cargoLock.lockFile = ./Cargo.lock;

            src = ./.;

            nativeBuildInputs = with pkgs; [
              openssl
              pkg-config
              rustPlatform.bindgenHook
            ];
            OPENSSL_DIR = "${pkgs.openssl.dev}";
            OPENSSL_LIB_DIR = "${pkgs.openssl.out}/lib";
            ORT_SKIP_DOWNLOAD = "1";
            ORT_LIB_LOCATION = "${onnxruntimeExtracted}/lib";
            ORT_INCLUDE_LOCATION = "${onnxruntimeExtracted}/include";
          };
        };
      });
}
