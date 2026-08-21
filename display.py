from tkinter import Tk, Canvas, PhotoImage


class Display:

    def __init__(self, data):
        self.window = Tk()

        self.window.title("Fly-in")
        self.window.attributes("-zoomed", True)
        self.window.config(bg="#021738")

        self.window.update()

        self.window_width = self.window.winfo_width()
        self.window_height = self.window.winfo_height()

        self.canvas = Canvas(
            self.window,
            width=self.window_width,
            height=self.window_height,
            bg="#021738"
        )

        self.canvas.pack()

        self.icon = PhotoImage(file="drone.png")
        self.window.iconphoto(True, self.icon)

        self.data = data

    def run(self):
        hubs, connections, nb_drones, neighbours = self.data

        hub_coordinates = {}
        x_coordinates = []
        y_coordinates = []

        for hub in hubs.values():
            hub_coordinates[hub["name"]] = {
                "x": hub["x"],
                "y": hub["y"]
            }

            x_coordinates.append(hub["x"])
            y_coordinates.append(hub["y"])

        x_range = abs(max(x_coordinates) - min(x_coordinates)) + 1
        y_range = abs(max(y_coordinates) - min(y_coordinates)) + 1

        x_coordinates.sort()
        y_coordinates.sort()

        x_unit = int(self.window_width / x_range)
        y_unit = int(self.window_height / y_range)

        zone_size = min(x_unit, y_unit) / 2

        x_canvas_coordinates = {}
        position = 0

        for coordinate in range(
            min(x_coordinates),
            max(x_coordinates) + 1
        ):
            if coordinate in x_coordinates:
                x_canvas_coordinates[coordinate] = position * x_unit

            position += 1

        y_canvas_coordinates = {}
        position = 0

        for coordinate in range(
            min(y_coordinates),
            max(y_coordinates) + 1
        ):
            if coordinate in y_coordinates:
                y_canvas_coordinates[coordinate] = position * y_unit

            position += 1

        canvas_objects = {}

        for hub_name, coordinates in hub_coordinates.items():

            x_position = x_canvas_coordinates[coordinates["x"]]
            y_position = y_canvas_coordinates[coordinates["y"]]

            canvas_object = self.canvas.create_oval(
                x_position,
                y_position,
                x_position + zone_size,
                y_position + zone_size,
                fill="red"
            )

            canvas_objects[hub_name] = canvas_object

        connection_coordinates = {}

        for connection_name, connection in connections.items():

            from_hub_coords = self.canvas.coords(
                canvas_objects[connection["from_hub"]]
            )

            from_x = from_hub_coords[0] + (
                from_hub_coords[2] - from_hub_coords[0]
            ) / 2

            from_y = from_hub_coords[1] + (
                from_hub_coords[3] - from_hub_coords[1]
            ) / 2

            to_hub_coords = self.canvas.coords(
                canvas_objects[connection["to_hub"]]
            )

            to_x = to_hub_coords[0] + (
                to_hub_coords[2] - to_hub_coords[0]
            ) / 2

            to_y = to_hub_coords[1] + (
                to_hub_coords[3] - to_hub_coords[1]
            ) / 2

            connection_coordinates[connection_name] = {
                "from_x": from_x,
                "from_y": from_y,
                "to_x": to_x,
                "to_y": to_y
            }

        for connection_name, coordinates in connection_coordinates.items():

            canvas_object = self.canvas.create_line(
                coordinates["from_x"],
                coordinates["from_y"],
                coordinates["to_x"],
                coordinates["to_y"],
                fill="blue",
                width=3,
                dash=(5, 5)
            )

            canvas_objects[connection_name] = canvas_object

        self.window.mainloop()