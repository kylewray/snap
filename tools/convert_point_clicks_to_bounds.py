import sys


def print_bounds(points):
    bounds = [[points['top-left'][0], points['top-left'][1]],
              [points['bottom-right'][0], points['top-left'][1]],
              [points['bottom-right'][0], points['bottom-right'][1]],
              [points['top-left'][0], points['bottom-right'][1]]]
    print(bounds)


def parse(filename):
    with open(filename, 'r') as f:
        assign = "top-left"
        points = {'top-left': [0.0, 0.0], 'bottom-right': [0.0, 0.0]}

        for line in f:
            line = line.strip()

            if line == "---":
                if assign == "top-left":
                    assign = "bottom-right"
                elif assign == "bottom-right":
                    assign = "top-left"
                    print_bounds(points)

            else:
                data = [d.strip() for d in line.split(":")]
                if len(data) != 2:
                    continue

                if data[0] == "x":
                    points[assign][0] = float(data[1])
                elif data[0] == "y":
                    points[assign][1] = float(data[1])


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please specify an output file to parse.")

    parse(sys.argv[1])

