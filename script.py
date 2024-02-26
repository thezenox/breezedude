Import("env")
import tools.uf2conv as uf2
from pathlib import Path

def make_uf2(source, target, env):
    print(target[0].get_abspath())

    with open(target[0].get_abspath(), mode='rb') as f:
        inpbuf = f.read()
        outbuf = uf2.convert_to_uf2(inpbuf)
        out = env['PROJECT_DIR'] +'\\build\\'
        Path(out).mkdir(parents=True, exist_ok=True)
        uf2.write_file(out + str(env["UNIX_TIME"]) + ".uf2", outbuf)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", make_uf2)

