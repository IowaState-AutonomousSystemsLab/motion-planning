from Tag import Tag
from Position import Position
import csv


def construct_tranfrm_csv(id, x, y, position_number, rotation):
    """
        Explanation
    """
    pass


if __name__ == '__main__':

    tags = []
    with open("RawTagLocations.csv", mode='r') as raw_tag_locations:
        raw_tag_location_reader = csv.reader(raw_tag_locations)

        line_count = 0
        for row in raw_tag_location_reader:
            if line_count > 0:
                id = int(row[0])
                x = int(row[1])
                y = int(row[2])
                position = row[3]
                if position == 1:
                    position = Position.top_left
                elif position == 2:
                    position = Position.top_right
                elif position == 3:
                    position = Position.bot_right
                elif position == 4:
                    position = Position.bot_left
                else:
                    print(f"[CSV READ ERR]: Row {line_count} has position value other than the rage 1-4")

                orientation = int(row[4])

                tags.append(Tag(id, (x, y), position, orientation))

                line_count += 1

    with open("TagLocations_cm.csv", "w") as tag_locations:
        tag_location_writer = csv.writer(tag_locations, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        tag_location_writer.writerow(["Tag ID", "x_cm", "y_cm", "theta"])
        for tag in tags:
            tag_location_writer.writerow(tag.get_position())
