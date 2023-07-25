import csv
import numpy as np
from enum import Enum

TILE_LENGTH_CM = 57.
DISTANCE_FROM_TILE_EDGE_CM = 2.


class Position(Enum):
    NorthWest = "nw"
    NorthEast = "ne"
    SouthWest = "sw"
    SouthEast = "se"


class Tag:
    id: int  # id of the april tag
    position: Position  # position of tag on the tile
    degrees: int  # the rotation of the tag in degrees from the origin.
    # (a tag facing east in the classroom is 0, then goes clockwise)
    world_tile: np.ndarray  # a vector of np array in the form of [[x],[y],[z]] where

    # x is the north-south direction of the class
    # y would be elevation, but is unused
    # z is the east-west direction of the class
    # the southeast corner of the lab is [[0],[y],[0]] and increases north and west

    # global_position: np.ndarray  # a vector of np array in the form of [[x],[y],[z]]

    def __init__(self, april_tag_id: int, tile_position: Position, degrees: int, world_tile: np.ndarray):
        self.id: int = april_tag_id
        self.position: Position = tile_position
        self.degrees: int = degrees
        self.world_tile: np.ndarray = world_tile

    def get_tag_location(self):
        """
        gets the tag's location in meters
        :return: the location of the tag in meters as a np vector in the form np.array( [[x],[y],[z]] )
        """


def read_csv(filename) -> [Tag]:
    tags = []
    with open(filename) as csvfile:
        tag_reader = csv.reader(csvfile)

        # skip header row
        next(tag_reader)

        for row in tag_reader:
            tag_id = int(row[0])
            tag_x = int(row[1])
            tag_y = int(row[2])
            tile_position = Position(row[3])
            degrees = int(row[4])

            # tag y in this representation is actually the z axis in the coordinate system
            location = np.array([[tag_x], [1], [tag_y]])
            t = Tag(tag_id, tile_position, degrees, location)
            # print(f"{t.id},{t.position},{t.degrees},{t.world_tile}")
            # print(t)
            tags.append(t)

    return tags.sort(key=lambda x: x.id)
