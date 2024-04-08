import subprocess
import os


def generate_rtlil(lib_path, driver_path):
    files = os.listdir(lib_path)
    # Only keep files that ends with .v
    files = [ lib_path+"/"+file for file in files if file.endswith(".v") ]

    # Include driver path
    files.append(driver_path)

    main_cmd_str = "plugin -i systemverilog; "
    for fname in files:
        main_cmd_str = main_cmd_str + "read_systemverilog -defer " + fname + "; "
    # file_names = " ".join(files)

    # cmd = ["yosys", "-p", "plugin -i systemverilog; read_systemverilog "
    #     + file_names + "; write_rtlil top.rtlil"]

    main_cmd_str += "read_systemverilog -link; synth -noabc -top top; opt -full -purge "
    # main_cmd_str += "write_rtlil top.rtlil"
    cmd = ["yosys", "-p", main_cmd_str]

    print(cmd)

    subprocess.run(cmd)
