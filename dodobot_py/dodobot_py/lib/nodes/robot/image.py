import io
from PIL import Image

def bytes_from_file(path, size, quality=15):
    img = Image.open(path)
    img = img.resize(size)

    img_byte_arr = io.BytesIO()
    img.save(img_byte_arr, format="JPEG", quality=quality)
    return img_byte_arr.getvalue()
