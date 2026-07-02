import os
import gzip

def compress_file(src_file: str, dest_file: str):
    data = bytes(0)
    with open(src_file, "rb") as file:
        for line in file:
            data += line

    dest_path_extension = dest_file.split('.')
    if dest_path_extension[-1] != "gz":
        dest_file += ".gz"

    with gzip.open(dest_file, "wb") as file:
        file.write(data)

    src_file_size = os.stat(src_file).st_size
    dest_file_size = os.stat(dest_file).st_size

    return src_file_size, dest_file_size


def compress_website(src_dir = "website", dest_dir = ".pio/build/website", ignore_files = []):
    print("Compressing website...")
    print("Website path: " + os.path.abspath(src_dir))
    
    files = [f for f in os.listdir(src_dir) if os.path.isfile(os.path.join(src_dir, f))]

    for ignore_file in ignore_files:
        if ignore_file in files:
            files.remove(ignore_file)

    os.makedirs(dest_dir, exist_ok=True)

    for file in files:
        file_path = os.path.join(src_dir, file)
        dest_file = os.path.join(dest_dir, file)

        s, d, = compress_file(file_path, dest_file)
        compression = (s / d)
        print(f"{file} size: {s}, compressed: {d}, compression: {compression}")

    print("Compressed website: " + os.path.abspath(dest_dir))
    print("Compressing done.")
