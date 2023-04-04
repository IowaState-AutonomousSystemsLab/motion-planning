from enum import Enum
from Position import Position

TILE_LENGTH_CM = 57.
DISTANCE_FROM_TILE_EDGE_CM = 2.

class Tag:

    def __init__(self, april_tag_id: int, coord: tuple[int, int], position: Position, rotation: int):
        self.id = april_tag_id
        self.coord = coord
        self.tag_x_pos = coord[0]
        self.tag_y_pos = coord[1]
        self.position = position
        self.rotation = rotation

    def get_position(self):
        """ Returns the position as a (x, y) in cm and theta in degrees

        Calculates it given the
            april_tag_id,
            coordinate of the tile the tag is on in x,y in tiles
            the position of the tag in the tile
            rotation in degrees. 0 is the TV at the front of the room (for Auto Lab at ISU)

            # >>> Tag(380, (4, 1), Position.top_right, 135)

        """

        if self.position == Position.top_left:
            x = self.tag_x_pos * TILE_LENGTH_CM + DISTANCE_FROM_TILE_EDGE_CM
            y = (self.tag_y_pos + 1) * TILE_LENGTH_CM
        elif self.position == Position.top_right:
            x = (self.tag_x_pos + 1) * TILE_LENGTH_CM - 2 * DISTANCE_FROM_TILE_EDGE_CM
            y = (self.tag_y_pos + 1) * TILE_LENGTH_CM - 2 * DISTANCE_FROM_TILE_EDGE_CM
        elif self.position == Position.bot_right:
            x = (self.tag_x_pos + 1) * TILE_LENGTH_CM - 2 * DISTANCE_FROM_TILE_EDGE_CM
            y = self.tag_y_pos * TILE_LENGTH_CM
        elif self.position == Position.bot_left:
            x = self.tag_x_pos * TILE_LENGTH_CM
            y = self.tag_y_pos * TILE_LENGTH_CM
        else:
            x = None
            y = None
            Exception("UNKNOWN POSITION WITHIN TILE")

        return self.id, x, y, self.rotation
