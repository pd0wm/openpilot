{
  description = "openpilot development environment (scons build via uv + vendored deps)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
  };

  outputs = { self, nixpkgs }:
    let
      # openpilot's native build runs on Linux (x86_64/aarch64). macOS is best-effort.
      linuxSystems = [ "x86_64-linux" "aarch64-linux" ];
      darwinSystems = [ "x86_64-darwin" "aarch64-darwin" ];
      allSystems = linuxSystems ++ darwinSystems;

      forAllSystems = f: nixpkgs.lib.genAttrs allSystems (system: f system);

      pkgsFor = system: import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };

      # Everything the prebuilt commaai/dependencies wheels, uv's managed CPython
      # and the scons C/C++ build expect to find in a normal FHS layout.
      targetPkgs = pkgs: with pkgs; [
        # --- build toolchain (mirrors `build-essential` from setup_dependencies.sh) ---
        # openpilot's SConstruct pins Environment(ENV={...}) to a fixed dict, so
        # env vars like NIX_CFLAGS_COMPILE never reach the build's gcc. Bake the
        # FHS /usr include path into the cc-wrapper instead, so system headers
        # (e.g. GL/gl.h pulled in by Qt for cabana) are found with no extra flags.
        # -idirafter keeps it lowest priority, below vendored and toolchain headers.
        (gcc.override {
          extraBuildCommands = ''
            echo "-idirafter /usr/include" >> $out/nix-support/cc-cflags
            echo "-L/usr/lib" >> $out/nix-support/cc-ldflags
          '';
        })
        gnumake
        binutils
        pkg-config
        coreutils
        findutils
        gnused
        gnugrep
        gawk
        which
        file
        bashInteractive

        # C/C++ runtime + system headers (linux/can.h, spidev.h, videodev2.h, ...)
        # NB: use the `.lib` output only — pulling in the full `stdenv.cc.cc`
        # package would shadow the nix cc-wrapper's gcc/g++ in /usr/bin with the
        # unwrapped compiler (which can't find glibc/crt or honor NIX_CFLAGS).
        stdenv.cc.cc.lib       # libstdc++ / libgomp / libgcc_s
        glibc
        linuxHeaders

        # --- python + package manager ---
        uv                     # installs all python deps incl. vendored native wheels
        python312              # uv pulls its own managed 3.12, this is just a fallback

        # --- core system deps from setup_dependencies.sh ---
        curl                   # libcurl4-openssl-dev (curl-config + headers)
        openssl
        git
        git-lfs
        cacert
        glibcLocales

        # --- Qt5: only needed for tools/cabana (skipped if qmake is absent) ---
        qt5.qtbase             # qmake/moc/rcc/uic + Qt5Core/Gui/Widgets
        qt5.qtcharts           # Qt5Charts
        qt5.qtsvg              # runtime SVG plugin for the bootstrap-icons assets
        libva                  # cabana links va/va-drm (also bundled in the ffmpeg wheel)

        # --- compression libs (also vendored, harmless to provide at system level) ---
        zlib
        bzip2
        zstd
        xz
        lz4

        # tinygrad compiles the driving/DM models at build time with DEV=CPU:LLVM
        # (selfdrive/modeld/SConscript); it dlopen()s libLLVM.so from /usr/lib.
        llvmPackages.libllvm

        # --- misc libs pulled in by python extension wheels ---
        libffi
        ncurses
        readline
        sqlite
        libxml2
        libusb1
        portaudio              # sounddevice (micd/soundd)
        dbus

        # --- graphics: raylib UI, replay, jotpluggler link against system GL/X11 ---
        libGL
        libGLU
        libglvnd
        mesa                   # EGL / gbm
        libdrm
        wayland
        libxkbcommon
        libx11
        libxrandr
        libxi
        libxcursor
        libxinerama
        libxext
        libxfixes
        libxcb
        fontconfig
        freetype
      ];

      mkFhsEnv = system:
        let pkgs = pkgsFor system;
        in (pkgs.buildFHSEnv {
          name = "openpilot-dev";
          inherit targetPkgs;
          # include dev (headers, *.pc) and lib outputs, not just the default `out`
          extraOutputsToInstall = [ "dev" "lib" ];

          profile = ''
            export LOCALE_ARCHIVE=${pkgs.glibcLocales}/lib/locale/locale-archive
            export LANG=en_US.UTF-8
            export LC_ALL=en_US.UTF-8
            export SSL_CERT_FILE=${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt
            export GIT_SSL_CAINFO="$SSL_CERT_FILE"
            # scons imports numpy from the repo root; make `openpilot.*` importable too
            export PYTHONPATH="$PWD''${PYTHONPATH:+:$PYTHONPATH}"

            # Let pkg-config find the FHS /usr libs (the cc-wrapper already has the
            # /usr include + lib paths baked in, so plain gcc/g++ work with no flags).
            export PKG_CONFIG_PATH="/usr/lib/pkgconfig:/usr/share/pkgconfig''${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"

            # Auto-install python deps and activate the venv on shell entry.
            # Set OPENPILOT_SKIP_SYNC=1 to skip the sync (e.g. when offline).
            if [ -f "$PWD/pyproject.toml" ]; then
              if [ -z "''${OPENPILOT_SKIP_SYNC:-}" ]; then
                echo "openpilot: uv sync --frozen --all-extras ..."
                uv sync --frozen --all-extras || echo "openpilot: uv sync failed (offline?), continuing without it"
              fi
              if [ -f "$PWD/.venv/bin/activate" ]; then
                source "$PWD/.venv/bin/activate"
                echo "openpilot: .venv activated. Build with:  scons -j\$(nproc)"
              fi
            fi
          '';

          runScript = "bash";
        });

      mkDarwinShell = system:
        let pkgs = pkgsFor system;
        in pkgs.mkShell {
          packages = with pkgs; [ uv git git-lfs pkg-config python312 ];
          shellHook = ''
            echo "openpilot dev shell (macOS, best-effort)."
            if [ -f "$PWD/pyproject.toml" ] && [ -z "''${OPENPILOT_SKIP_SYNC:-}" ]; then
              echo "openpilot: uv sync --frozen --all-extras ..."
              uv sync --frozen --all-extras || echo "openpilot: uv sync failed, continuing"
            fi
            [ -f "$PWD/.venv/bin/activate" ] && source "$PWD/.venv/bin/activate"
            echo "Build with: scons -j\$(sysctl -n hw.ncpu)"
          '';
        };
    in
    {
      devShells = forAllSystems (system:
        {
          default =
            if builtins.elem system linuxSystems
            then (mkFhsEnv system).env
            else mkDarwinShell system;
        });

      # Runnable FHS wrapper: `nix run .#openpilot-dev -- -c 'scons -j8'`
      # runs a command inside the same sandbox as the dev shell.
      packages = nixpkgs.lib.genAttrs linuxSystems (system: {
        openpilot-dev = mkFhsEnv system;
        default = mkFhsEnv system;
      });
    };
}
