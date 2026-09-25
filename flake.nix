{
  description = "Reproducible standalone Fortran FABRIK core with flat C ABI";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      makePackage = { stdenv, gfortran, fortran-fpm, lib }:
        stdenv.mkDerivation {
          pname = "fabrik-core";
          version = "0.1.0";
          # A path flake must not copy local FPM output into the source. Stale
          # objects can otherwise survive a rebuild and link an obsolete C ABI.
          src = lib.cleanSourceWith {
            src = ./.;
            name = "fabrik-core-source";
            filter = path: type:
              let base = baseNameOf (toString path);
              in !(builtins.elem base [ ".git" "build" ".godot" ]);
          };
          strictDeps = true;
          nativeBuildInputs = [ gfortran fortran-fpm ];
          dontConfigure = true;
          buildPhase = ''
            runHook preBuild
            export PATH="${fortran-fpm}/bin:$PATH"
            fortran-fpm build --profile release
            runHook postBuild
          '';
          doCheck = true;
          checkPhase = ''
            runHook preCheck
            export PATH="${fortran-fpm}/bin:$PATH"
            fortran-fpm test --profile release
            runHook postCheck
          '';
          installPhase = ''
            runHook preInstall
            install -Dm644 include/fabrik_core.h $out/include/fabrik_core.h
            install -Dm644 "$(find build -name libfabrik_core.a -type f | head -1)" $out/lib/libfabrik_core.a
            runHook postInstall
          '';
          passthru = {
            tests = "4 Fortran solver tests: convergence, unreachable, degenerate, determinism";
            cAbi = "flat C ABI; no derived-type ABI leakage";
          };
        };
      packageFor = system:
        let pkgs = nixpkgs.legacyPackages.${system}; in
        makePackage {
          stdenv = pkgs.stdenv;
          gfortran = pkgs.gfortran;
          fortran-fpm = pkgs.fortran-fpm;
          lib = pkgs.lib;
        };
    in {
      packages = nixpkgs.lib.genAttrs systems (system: {
        default = packageFor system;
      });
      checks = nixpkgs.lib.genAttrs systems (system: {
        default = self.packages.${system}.default;
      });
      devShells = nixpkgs.lib.genAttrs systems (system:
        let pkgs = nixpkgs.legacyPackages.${system}; in {
          default = pkgs.mkShell {
            packages = [ pkgs.gfortran pkgs.fortran-fpm pkgs.gnumake ];
          };
        });
      formatter = nixpkgs.lib.genAttrs systems (system:
        nixpkgs.legacyPackages.${system}.nixpkgs-fmt);
    };
}
