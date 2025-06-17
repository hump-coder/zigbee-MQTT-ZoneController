Import("env")

src_dir = env.GetProjectOption("custom_src_dir")
if src_dir:
    env.Replace(PROJECT_SRC_DIR=env.Dir(src_dir).abspath)
