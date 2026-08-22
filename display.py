from tkinter import Tk, Canvas, PhotoImage, Button


class Display:

    def __init__(self, data, zones_at_turnes):

        self.data = data
        self.zones_at_turnes = zones_at_turnes

        self.current_turn = 0
        self.zone_text_objects = {}

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

        self.txt_obj = self.canvas.create_text(
            self.window_width - 100,
            50,
            text="turn: 0",
            font=("courier", 16, "bold"),
            fill="#5262d5",
            anchor="n"
        )

        self.icon = PhotoImage(file="drone.png")
        self.window.iconphoto(True, self.icon)

        self.window.bind("<Right>", self.next_turn)
        self.window.bind("<Left>", self.prev_turn)

    def _format_zone_text(self, drone_ids):

        text_content = str(drone_ids)

        return "\n".join(
            text_content[i:i + 10]
            for i in range(0, len(text_content), 10)
        )

    def _update_zone_texts(self):

        turn_data = self.zones_at_turnes[self.current_turn][1]

        for zone_name, drone_ids in turn_data.items():

            text_object = self.zone_text_objects[zone_name]
            formatted_text = self._format_zone_text(drone_ids)

            self.canvas.itemconfig(
                text_object,
                text=formatted_text
            )

    def next_turn(self, event=None):

        if self.current_turn < len(self.zones_at_turnes) - 1:

            self.current_turn += 1
            self._update_zone_texts()
            print("Current turn:", self.current_turn)

            self.canvas.itemconfig(
                self.txt_obj,
                text=f"turn: {self.current_turn}"
            )

    def prev_turn(self, event=None):

        if self.current_turn > 0:

            self.current_turn -= 1
            self._update_zone_texts()
            print("Current turn:", self.current_turn)

            self.canvas.itemconfig(
                self.txt_obj,
                text=f"turn: {self.current_turn}"
            )

    def _create_navigation_buttons(self):

        prev_button = Button(
            self.window,
            text="Prev",
            command=self.prev_turn,
            font=("courier", 14, "bold")
        )

        next_button = Button(
            self.window,
            text="Next",
            command=self.next_turn,
            font=("courier", 14, "bold")
        )

        next_button.place(
            relx=0.98,
            rely=0.95,
            anchor="se"
        )

        prev_button.place(
            relx=0.93,
            rely=0.95,
            anchor="se"
        )

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

        x_range = abs(
            max(x_coordinates) - min(x_coordinates)
        ) + 1

        y_range = abs(
            max(y_coordinates) - min(y_coordinates)
        ) + 1

        x_coordinates.sort()
        y_coordinates.sort()

        x_unit = int(
            self.window_width / x_range
        )
        y_unit = int(
            self.window_height / y_range
        )

        zone_size = 100
        x_canvas_coordinates = {}
        position = 0

        for coordinate in range(
            min(x_coordinates),
            max(x_coordinates) + 1
        ):

            if coordinate in x_coordinates:

                x_canvas_coordinates[coordinate] = (
                    position * x_unit + 30
                )

            position += 1

        y_canvas_coordinates = {}
        position = 0

        for coordinate in range(
            min(y_coordinates),
            max(y_coordinates) + 1
        ):

            if coordinate in y_coordinates:

                y_canvas_coordinates[coordinate] = (
                    position * y_unit + 30
                )

            position += 1
        canvas_objects = {}

        print("hub_coordinates", hub_coordinates)

        for hub_name, coordinates in hub_coordinates.items():

            x_position = (
                x_canvas_coordinates[coordinates["x"]]
            )
            y_position = (
                y_canvas_coordinates[coordinates["y"]]
            )

            nx_position = x_position + zone_size
            ny_position = y_position + zone_size

            canvas_object = self.canvas.create_oval(
                x_position,
                y_position,
                nx_position,
                ny_position,
                fill="#a699e8"
            )

            self.canvas.create_text(
                nx_position - (
                    (nx_position - x_position) / 2
                ),
                ny_position - zone_size - 20,
                text=hub_name,
                font=("courier", 16, "bold"),
                fill="#5262d5",
                anchor="n"
            )

            drone_ids = (
                self.zones_at_turnes[
                    self.current_turn
                ][1][hub_name]
            )

            formatted_text = self._format_zone_text(
                drone_ids
            )

            text_object = self.canvas.create_text(
                nx_position - (
                    (nx_position - x_position) / 2
                ),
                ny_position + 20,
                text=formatted_text,
                font=("courier", 16, "bold"),
                fill="#5262d5",
                anchor="n"
            )

            self.zone_text_objects[hub_name] = (
                text_object
            )

            canvas_objects[hub_name] = canvas_object

        connection_coordinates = {}

        for connection_name, connection in connections.items():

            from_hub_coords = self.canvas.coords(
                canvas_objects[
                    connection["from_hub"]
                ]
            )

            from_x = from_hub_coords[0] + (
                from_hub_coords[2]
                - from_hub_coords[0]
            ) / 2

            from_y = from_hub_coords[1] + (
                from_hub_coords[3]
                - from_hub_coords[1]
            ) / 2

            to_hub_coords = self.canvas.coords(
                canvas_objects[
                    connection["to_hub"]
                ]
            )

            to_x = to_hub_coords[0] + (
                to_hub_coords[2]
                - to_hub_coords[0]
            ) / 2

            to_y = to_hub_coords[1] + (
                to_hub_coords[3]
                - to_hub_coords[1]
            ) / 2

            connection_coordinates[connection_name] = {
                "from_x": from_x,
                "from_y": from_y,
                "to_x": to_x,
                "to_y": to_y
            }

        for connection_name, coordinates in (
            connection_coordinates.items()
        ):

            canvas_object = self.canvas.create_line(
                coordinates["from_x"],
                coordinates["from_y"],
                coordinates["to_x"],
                coordinates["to_y"],
                fill="#1ddbd5",
                width=3,
                dash=(5, 5),
                tags="connection"
            )

            canvas_objects[
                connection_name
            ] = canvas_object

            self.canvas.tag_lower("connection")

        self._create_navigation_buttons()

        self.window.mainloop()