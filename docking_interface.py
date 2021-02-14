import krpc
import time

conn = krpc.connect(name='DockingUI')

vessel = conn.space_center.active_vessel
target = conn.space_center.target_docking_port
canvas = conn.ui.stock_canvas
screen_size = canvas.rect_transform.size

panel = canvas.add_panel()

rect = panel.rect_transform
rect.size = (310, 110)
rect.position = (rect.size[0]*.55-(screen_size[0]/2), 0)

ui_text = panel.add_text("Offset: x, y, z")
ui_text.rect_transform.position = (0, 0)
ui_text.rect_transform.size = (300, 100)
ui_text.alignment = conn.ui.TextAnchor.middle_left
ui_text.color = (1, 1, 1)
ui_text.size = 20
print('Begin')
while True:
    position = vessel.parts.controlling.position(target.reference_frame)
    velocity = vessel.parts.controlling.velocity(target.reference_frame)
    ui_string = f'Offset:  {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}\n\n' \
                f'Velocity:{velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}'
    ui_text.content = ui_string
    time.sleep(.05)
