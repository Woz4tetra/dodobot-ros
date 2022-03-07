import pygame
from PIL import Image, ImageDraw


def method_dispose(frames, previous_frame):
    # from: https://stackoverflow.com/questions/60890497/python-pillow-gif-artifacts-background
    # 0 PIL = Overlay and pass
    # 1 PIL = Overlay and return previous
    # 2 PIL = Erase Overlay
    # 3 PIL = Restore to previous
    # 4-7 PIL = undefined
    if previous_frame is None:
        previous_frame = Image.new('RGBA', size=frames.size)
    new_frame = previous_frame.copy()
    current_frame = frames.convert('RGBA')
    new_frame.alpha_composite(current_frame, dest=frames.dispose_extent[0:2], source=frames.dispose_extent)
    if frames.disposal_method == 0:
        return new_frame, Image.new('RGBA', frames.size)
    elif frames.disposal_method == 1:
        return new_frame, new_frame.copy()
    elif frames.disposal_method == 2:
        draw = ImageDraw.Draw(previous_frame)
        x0, y0, x1, y1 = frames.dispose_extent
        draw.rectangle((x0, y0, x1 - 1, y1 - 1), fill=(255, 255, 255, 255), width=0)  # fill white transparent
        return new_frame, previous_frame.copy()
    elif frames.disposal_method == 3:
        return new_frame, previous_frame.copy()


def to_pygame_gif(gif_array):
    pygame_gif = []
    for frame, duration in gif_array:
        pygame_image = pygame.image.fromstring(
            frame.tobytes(), frame.size, frame.mode
        )
        pygame_gif.append((pygame_image, duration))
        pygame_image.get_width()
    return pygame_gif


def load_gif(path, use_pygame=True):
    gif = Image.open(path)
    gif_array = []
    pass_frame = None
    for frame_index in range(gif.n_frames):
        gif.seek(frame_index)
        disp_frame, pass_frame = method_dispose(gif, pass_frame)

        duration = gif.info["duration"]
        gif_array.append((disp_frame, duration))

        gif.seek(0)

    if use_pygame:
        return gif, to_pygame_gif(gif_array)
    else:
        return gif, gif_array


if __name__ == '__main__':
    def convert_gif(path):
        import os
        gif, frames = load_gif(path, False)

        images = []
        for index, frame in enumerate(frames):
            im = frame[0]
            images.append(im)
            # im = im.convert('RGB')

            # im.save("out-%s.png" % index)

        # images = []
        # for filename in os.listdir("."):
        #     if filename.startswith("out"):
        #         print(filename)
        #         start = len("out-")
        #         stop = filename.find(".")
        #         index = int(filename[start:stop])
        #         while len(images) <= index:
        #             images.append(None)
        #         images[index] = Image.open(filename)
        images[0].save("dest.gif", save_all=True, append_images=images, duration=frames[0][1], loop=0, disposal=1)
    convert_gif("/Users/Woz4tetra/Desktop/chansey.gif")