from tkinter import *

window = Tk()

window.title("Fly-in")
# window.geometry("1600x900")
window.attributes("-zoomed", True)
window.config(bg="#021738")
window.update()
w = window.winfo_width()
h = window.winfo_height()

canvas = Canvas(window, width=w, height=h, bg="#021738")
canvas.pack()

icon = PhotoImage(file="drone.png")
window.iconphoto(True, icon)

def mainloop(data):
    hubs, connections, nb_drones, neighbours = data
    cords = {}
    x_cords = []
    y_cords = []
    for v in hubs.values():
        cords[v['name']] = {"x": v['x'], "y": v['y']}
        x_cords.append(v['x'])
        y_cords.append(v['y'])
    print(cords)
    x_range = abs(max(x_cords) - min(x_cords)) + 1
    y_range = abs(max(y_cords) - min(y_cords)) + 1
    x_cords.sort()
    y_cords.sort()
    print(x_cords)
    print(y_cords)
    print(x_range)
    print(y_range)
    x_unit = int(w / x_range)
    y_unit = int(h / y_range)
    zone_width = min([x_unit, y_unit]) / 2
    print(x_unit)
    print(y_unit)
    print(zone_width)
    x_new_cords = {}
    x = 0
    for i in range(min(x_cords), max(x_cords) + 1):
        if i in x_cords:
            x_new_cords[i] = x * x_unit
        x+=1
    print(x_new_cords)
    y_new_cords = {}
    x = 0
    for i in range(min(y_cords), max(y_cords) + 1):
        if i in y_cords:
            y_new_cords[i] = x * y_unit
        x+=1
    print(y_new_cords)
    canvas_objs = {}
    for k, v in cords.items():
        x = x_new_cords[v['x']]
        y = y_new_cords[v['y']]
        canvas_obj = canvas.create_oval(x, y, x+zone_width, y+zone_width, fill="red")
        canvas_objs[k] = canvas_obj
    print(canvas_objs)
    for k, v in canvas_objs.items():
        print(k, canvas.coords(v))
    print("-------------------")
    connections_coords = {}
    for k, v in connections.items():
        from_hub = canvas.coords(canvas_objs[v['from_hub']])
        fx = from_hub[1] + ((from_hub[3] - from_hub[1]) / 2)
        fy = from_hub[0] + ((from_hub[2] - from_hub[0]) / 2)
        to_hub = canvas.coords(canvas_objs[v['to_hub']])
        tx = to_hub[1] + ((to_hub[3] - to_hub[1]) / 2)
        ty = to_hub[0] + ((to_hub[2] - to_hub[0]) / 2)
        connections_coords[k] = {"fx": fx , "fy": fy, "tx": tx, "ty": ty}
    print(connections_coords)
    for k, v in connections_coords.items():
        fx = v['fx']
        fy = v['fy']
        tx = v['tx']
        ty = v['ty']
        canvas_obj = canvas.create_line(fy, fx, ty, tx, fill="blue", width=3, dash=(5, 5))
        canvas_objs[k] = canvas_obj

    window.mainloop()