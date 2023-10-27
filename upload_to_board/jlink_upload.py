import subprocess

args = ['C:\\Program Files\\SEGGER\\JLink\\JLink.exe', '-device', 'max32660', '-if', 'SWD', '-speed', '4000', '-autoconnect', '1', '-commandfile', 'C:\\sensor_fusion\\upload_to_board\\command_file.jlink']
subprocess.call(args) 