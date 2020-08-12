"""This module implements an abstract base class (ABC) 'BaseDataset' for datasets.

It also includes common transformation functions (e.g., get_transform, __scale_width), which can be later used in
subclasses. """
import random
from typing import Tuple, Dict, Any

import numpy as np
import torch.utils.data as data
import torchvision.transforms as transforms
from PIL import Image
import PIL.ImageOps


class BaseDataset(data.Dataset):
    """This is the base class for other datasets.

    To create a subclass, you need to implement the following four functions:
    -- <__init__>:                      initialize the class, first call BaseDataset.__init__(self, opt).
    -- <__len__>:                       return the size of dataset.
    -- <__getitem__>:                   get a data point.
    """

    def __len__(self):
        """Return the total number of images in the dataset."""
        return -1

    def __getitem__(self, index):
        """Return a data point and its metadata information.

        Parameters:
            index - - a random integer for data indexing

        Returns:
            a dictionary of data with their names. It ususally contains the data itself and its metadata information.
        """
        pass


def get_params(
    preprocess: str, load_size: int, crop_size: int, size: Tuple[int, int]
) -> Dict[str, Any]:
    w, h = size
    new_h = h
    new_w = w
    if preprocess == "resize_and_crop":
        new_h = new_w = load_size
    elif preprocess == "scale_width_and_crop":
        new_w = load_size
        new_h = load_size * h // w

    x = random.randint(0, np.maximum(0, new_w - crop_size))
    y = random.randint(0, np.maximum(0, new_h - crop_size))

    flip = random.random() > 0.5

    return {"crop_pos": (x, y), "flip": flip}


def get_transform(
    load_size: int,
    crop_size: int,
    mask: str = None,
    preprocess: str = "none",
    no_flip: bool = True,
    params=None,
    grayscale=False,
    method=Image.BICUBIC,
    convert=True,
) -> transforms.Compose:
    """Create transformation from arguments."""
    transform_list = []

    if grayscale:
        transform_list.append(transforms.Grayscale(1))

    if mask is not None:
        transform_list.append(transforms.Lambda(lambda img: __apply_mask(img, mask)))
    if "resize" in preprocess:
        osize = [load_size, load_size]
        transform_list.append(transforms.Resize(osize, method))
    elif "scale_width" in preprocess:
        transform_list.append(
            transforms.Lambda(lambda img: __scale_width(img, load_size, crop_size, method))
        )

    if "crop" in preprocess:
        if params is None:
            transform_list.append(transforms.RandomCrop(crop_size))
        else:
            transform_list.append(
                transforms.Lambda(lambda img: __crop(img, params["crop_pos"], crop_size))
            )

    if preprocess == "none":
        transform_list.append(
            transforms.Lambda(lambda img: __make_power_2(img, base=4, method=method))
        )

    if not no_flip:
        if params is None:
            transform_list.append(transforms.RandomHorizontalFlip())
        elif params["flip"]:
            transform_list.append(
                transforms.Lambda(lambda img: __flip(img, params["flip"]))
            )

    if convert:
        transform_list += [transforms.ToTensor()]
        if grayscale:
            transform_list += [transforms.Normalize((0.5,), (0.5,))]
        else:
            transform_list += [transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))]
    return transforms.Compose(transform_list)


def __make_power_2(img, base, method=Image.BICUBIC):
    ow, oh = img.size
    h = int(round(oh / base) * base)
    w = int(round(ow / base) * base)
    if h == oh and w == ow:
        return img

    __print_size_warning(ow, oh, w, h)
    return img.resize((w, h), method)


def __scale_width(img, target_size, crop_size, method=Image.BICUBIC):
    ow, oh = img.size
    if ow == target_size and oh >= crop_size:
        return img
    w = target_size
    h = int(max(target_size * oh / ow, crop_size))
    return img.resize((w, h), method)


def __crop(img, pos, size):
    ow, oh = img.size
    x1, y1 = pos
    tw = th = size
    if ow > tw or oh > th:
        return img.crop((x1, y1, x1 + tw, y1 + th))
    return img


def __apply_mask(img: Image.Image, mask_file: str) -> Image.Image:
    """Overlay image with the provided mask."""
    mask = Image.open(mask_file)
    # Use inverted mask as the intensity of the masking. This means that white parts are see through.
    img.paste(mask, (0, 0), PIL.ImageOps.invert(mask))
    return img


def __flip(img, flip):
    if flip:
        return img.transpose(Image.FLIP_LEFT_RIGHT)
    return img


def __print_size_warning(ow, oh, w, h):
    """Print warning information about image size(only print once)"""
    if not hasattr(__print_size_warning, "has_printed"):
        print(
            "The image size needs to be a multiple of 4. "
            "The loaded image size was (%d, %d), so it was adjusted to "
            "(%d, %d). This adjustment will be done to all images "
            "whose sizes are not multiples of 4" % (ow, oh, w, h)
        )
        __print_size_warning.has_printed = True