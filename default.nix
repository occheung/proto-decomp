let
    pkgs = import <nixpkgs> { };
in with pkgs;
let
    pyrtl = python310Packages.buildPythonPackage rec {
        name = "PyRTL";

        src = fetchFromGitHub {
            owner = "pllab";
            repo = "PyRTL";
            # rev = "fe26f909c2bb1ed3851ccf9004730d6897726208";
            # sha256 = "Rgi9yrpmDA2LW2AmDY8XGZehS8nYqFnY4p4GH15yfmw=";
            rev = "7e2e8dd94d62c1a5e561e0a45de15aa44d1a8e04";
            sha256 = "Iqr0vK84zvP+WWj0ikzqhpppkW4+bnDh+jgQCgd9SI8=";
        };

        # Failing test
        # checkInputs = with python310Packages; [ nose tox ];

        doCheck = false;

        propagatedBuildInputs = with python310Packages; [ six pyparsing graphviz ];
    };

    migen = python310Packages.buildPythonPackage rec {
        name = "migen";

        src = fetchFromGitHub {
            owner = "m-labs";
            repo = "migen";
            rev = "50934ad10a87ade47219b796535978b9bdf24023";
            sha256 = "gKIfHZxsv+jcgDFRW9mPqmwqbZXuRvXefkZcSFjOGHw=";
        };

        format = "pyproject";
        nativeBuildInputs = [ pkgs.python3Packages.setuptools ];

        propagatedBuildInputs = with python310Packages; [ colorama ];
    };

    misoc = python310Packages.buildPythonPackage rec {
        name = "misoc";

        src = fetchFromGitHub {
            owner = "m-labs";
            repo = "misoc";
            rev = "a53859f2167c31ab5225b6c09f30cf05527b94f4";
            sha256 = "G5DktaZfyfIE2WhXRoV10DbGZObOo3Bw4AN1gbR7QjQ=";
        };

        propagatedBuildInputs = with python310Packages; [
            jinja2 numpy pyserial asyncserial migen
        ];
    };

    yosys-cxx17 = pkgs.yosys.overrideAttrs(oa: {
        makeFlags = oa.makeFlags ++ [ "ENABLE_LIBYOSYS=1" "CXXSTD=c++17" ];
        # We actually don't need ENABLE_LIBYOSYS

        doCheck = false;

        # Not much luck with the python port
        # nativeBuildInputs = oa.nativeBuildInputs ++ [ python310Packages.boost ];
    });

    # ys-script = writeShellScriptBin "yosys-c++" ''
    #     g++ $1 -I$SRC_YOSYS -D_YOSYS_ -L$LIB_YOSYS -lyosys
    # '';

in
    stdenv.mkDerivation {
        name = "decomp-env";
        buildInputs = [
            (pkgs.python3.withPackages(ps: [ pyrtl migen misoc ]))
            yosys-cxx17
            pkgs.verilator
            pkgs.racket
            pkgs.gtkwave
            pkgs.xdot
            pkgs.readline

            # ys-script
        ];

        # shellHook = ''
        #     export SRC_YOSYS=${yosys-fixed}/share/yosys/include
        #     export LIB_YOSYS=${yosys-fixed}/lib/yosys
        # '';
    }
