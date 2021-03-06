# This file was generated by the Tkinter Designer by Parth Jadhav
# https://github.com/ParthJadhav/Tkinter-Designer
from functools import partial
from pathlib import Path
import threading
import control_fn as cf
import rs_485 as com
import Arduino_port as ardcom
from tkinter import StringVar, Tk, Canvas, Entry, Button, PhotoImage

OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path("./assets")


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

if __name__ == '__main__':
    thread1 = threading.Thread(target=com.read_from_port, args=(com.ser,))
    thread1.start()
    thread2 = threading.Thread(target=ardcom.read_from_port, args=(ardcom.ard,))
    thread2.start()
    thread3 = threading.Thread(target=cf.mainprog)
    thread3.start()

    window = Tk()
    window.geometry("1240x1024")
    window.configure(bg="#FFFFFF")

    id = StringVar()
    angle = StringVar()
    speed = StringVar()

    _x = StringVar()
    _y = StringVar()
    _z = StringVar()
    _alp = StringVar()
    _bel = StringVar()
    _gam = StringVar()

    canvas = Canvas(
        window,
        bg="#FFFFFF",
        height=1024,
        width=1240,
        bd=0,
        highlightthickness=0,
        relief="ridge"
    )

    canvas.place(x=0, y=0)
    button_image_1 = PhotoImage(
        file=relative_to_assets("button_1.png"))
    btn_set0 = Button(
        image=button_image_1,
        borderwidth=0,
        highlightthickness=0,
        # command=runIncMm,
        relief="flat"
    )
    btn_set0.place(
        x=98.0,
        y=464.0,
        width=200.0,
        height=48.0
    )

    button_go0 = PhotoImage(
        file=relative_to_assets("button_go0.png"))

    btn_go0 = Button(
        image=button_go0,
        borderwidth=0,
        highlightthickness=0,
        command=cf.run_to0,
        relief="flat"
    )
    btn_go0.place(
        x=98.0,
        y=536.0,
        width=200.0,
        height=48.0
    )
    button_stop = PhotoImage(
        file=relative_to_assets("button_stop.png"))

    btn_stop = Button(
        image=button_stop,
        borderwidth=0,
        highlightthickness=0,
        command=cf.stop_motor,
        relief="flat"
    )
    btn_stop.place(
        x=348.0,
        y=536.0,
        width=200.0,
        height=48.0
    )

    button_image_2 = PhotoImage(
        file=relative_to_assets("button_2.png"))
    btn_run = Button(
        image=button_image_2,
        borderwidth=0,
        highlightthickness=0,
        command=partial(cf.run_motion, _x, _y, _z, _alp, _bel, _gam),
        relief="flat"
    )
    btn_run.place(
        x=348.0,
        y=464.0,
        width=200.0,
        height=48.0
    )

    canvas.create_rectangle(
        860.0,
        150.0,
        1110.0,
        512.0,
        fill="#EBEBEB",
        outline="")

    button_image_3 = PhotoImage(
        file=relative_to_assets("button_3.png"))

    btn_plus = Button(
        image=button_image_3,
        borderwidth=0,
        highlightthickness=0,
        command=partial(cf.runInc, 1, id, angle, speed),
        relief="flat"
    )
    btn_plus.place(
        x=885.0,
        y=258.0,
        width=200.0,
        height=48.0
    )

    button_image_4 = PhotoImage(
        file=relative_to_assets("button_4.png"))
    btn_minus = Button(
        image=button_image_4,
        borderwidth=0,
        highlightthickness=0,
        command=partial(cf.runInc, 2,id, angle, speed),
        relief="flat"
    )
    btn_minus.place(
        x=885.0,
        y=366.0,
        width=200.0,
        height=48.0
    )

    entry_id_img = PhotoImage(
        file=relative_to_assets("entry_9.png"))
    entry_bg_9 = canvas.create_image(
        198.0,
        174.0,
        image=entry_id_img
    )
    entry_id = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=id
    )
    entry_id.place(
        x=108.0,
        y=150.0,
        width=180.0,
        height=46.0
    )

    entry_angle_img = PhotoImage(
        file=relative_to_assets("entry_8.png"))
    entry_bg_8 = canvas.create_image(
        442.0,
        174.0,
        image=entry_angle_img
    )
    entry_angle = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=angle
    )
    entry_angle.place(
        x=352.0,
        y=150.0,
        width=180.0,
        height=46.0
    )

    entry_speed_img = PhotoImage(
        file=relative_to_assets("entry_7.png"))
    entry_bg_7 = canvas.create_image(
        686.0,
        174.0,
        image=entry_speed_img
    )
    entry_speed = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=speed
    )
    entry_speed.place(
        x=596.0,
        y=150.0,
        width=180.0,
        height=46.0
    )

    canvas.create_text(
        860.0,
        124.0,
        anchor="nw",
        text="Jog",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        592.0,
        335.0,
        anchor="nw",
        text="Gama",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        348.0,
        335.0,
        anchor="nw",
        text="Belta",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        104.0,
        335.0,
        anchor="nw",
        text="Alpha",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        586.0,
        227.0,
        anchor="nw",
        text="Z",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        342.0,
        227.0,
        anchor="nw",
        text="Y",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        98.0,
        227.0,
        anchor="nw",
        text="X",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        586.0,
        119.0,
        anchor="nw",
        text="Speed",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        342.0,
        119.0,
        anchor="nw",
        text="Angle",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    canvas.create_text(
        98.0,
        119.0,
        anchor="nw",
        text="ID",
        fill="#000000",
        font=("Inter Light", 20 * -1)
    )

    entry_game_img = PhotoImage(
        file=relative_to_assets("entry_1.png"))
    entry_bg_1 = canvas.create_image(
        692.0,
        390.0,
        image=entry_game_img
    )
    entry_game = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=_gam
    )
    entry_game.place(
        x=602.0,
        y=366.0,
        width=180.0,
        height=46.0
    )

    entry_belta_img = PhotoImage(
        file=relative_to_assets("entry_2.png"))
    entry_bg_2 = canvas.create_image(
        448.0,
        390.0,
        image=entry_belta_img
    )
    entry_belta = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=_bel
    )
    entry_belta.place(
        x=358.0,
        y=366.0,
        width=180.0,
        height=46.0
    )

    entry_alpha_img = PhotoImage(
        file=relative_to_assets("entry_3.png"))
    entry_bg_3 = canvas.create_image(
        204.0,
        390.0,
        image=entry_alpha_img
    )
    entry_alpha = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=_alp

    )
    entry_alpha.place(
        x=114.0,
        y=366.0,
        width=180.0,
        height=46.0
    )

    entry_z_img = PhotoImage(
        file=relative_to_assets("entry_4.png"))
    entry_bg_4 = canvas.create_image(
        686.0,
        282.0,
        image=entry_z_img
    )
    entry_z = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=_z
    )
    entry_z.place(
        x=596.0,
        y=258.0,
        width=180.0,
        height=46.0
    )

    entry_y_img = PhotoImage(
        file=relative_to_assets("entry_5.png"))
    entry_bg_5 = canvas.create_image(
        442.0,
        282.0,
        image=entry_y_img
    )
    entry_y = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=_y
    )
    entry_y.place(
        x=352.0,
        y=258.0,
        width=180.0,
        height=46.0
    )

    entry_x_img = PhotoImage(
        file=relative_to_assets("entry_6.png"))
    entry_bg_6 = canvas.create_image(
        198.0,
        282.0,
        image=entry_x_img
    )
    entry_x = Entry(
        bd=0,
        bg="#EBEBEB",
        highlightthickness=0,
        textvariable=_x
    )
    entry_x.place(
        x=108.0,
        y=258.0,
        width=180.0,
        height=46.0
    )

    canvas.create_rectangle(
        0.0,
        0.0,
        1440.0,
        64.0,
        fill="#7A8DD1",
        outline="")

    canvas.create_text(
        98.0,
        13.0,
        anchor="nw",
        text="Test Robot",
        fill="#000000",
        font=("Inter ExtraLight", 32 * -1)
    )
    window.resizable(False, False)
    window.mainloop()
