"""
V3 additions to the AWS hospital world: interior lights, laboratory furnishing, collection table + payload,
logistics plaques, directional signs and the dynamic traffic models. Positions come from v3_hospital_layout.py
(AWS hospital frame = map frame, metres). Sign textures are generated in the V2.6 sign style (white board, blue
border, text + arrow) so the existing V3 YOLO 'directional_sign' class applies unchanged.
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

import v3_hospital_layout as L

V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIGN_DIR = os.path.join(V3, "hospital", "signs")
MARKER_DIR = os.path.join(V3, "models", "v3_markers")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# Ceiling-light substitutes (the AWS ceiling is left out so the GUI shows the plan from above)
LIGHTS = [(0, 13), (0, 6), (-5, 0), (5, 0), (-5, -10), (5, -10), (0, -14.5), (-5, -20), (5, -20), (0, -24.5),
          (-9.5, -17), (9.5, -17), (9.4, 14.6), (-9.4, 14.6), (-9.5, -4), (9.5, -4), (5.5, -29), (-9.5, -28)]

# Laboratory furnishing (NE room x 6.1..12.45, y 12.15..16.85) and the collection table in the west ward:
# (name, model, x, y, yaw)
PROPS = [
    ("v3_lab_cabinet_1", "MetalCabinet", 12.05, 15.9, -1.5708),
    ("v3_lab_cabinet_2", "MetalCabinet", 12.05, 14.9, -1.5708),
    ("v3_lab_drawer_1", "Drawer", 12.1, 13.6, -1.5708),
    ("v3_lab_drawer_2", "Drawer", 12.1, 13.0, -1.5708),
    ("v3_lab_instrument_cart", "InstrumentCart2", 11.1, 12.6, 0.0),
    ("v3_lab_bp_monitor", "BloodPressureMonitor", 9.9, 12.55, 3.1416),
    ("v3_lab_storage", "StorageRackCovered", 10.75, 16.35, 0.0),
    ("v3_ward_collection_table", "AdjTable", -7.15, -16.6, 1.5708),
]


def bench_sdf(name, x, y, length, yaw=0.0):
    """Laboratory work bench: grey cabinet body, white worktop (0.9 m), as a static box model."""
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
      <link name='link'>
        <collision name='body'><pose>0 0 0.44 0 0 0</pose><geometry><box><size>{length} 0.7 0.88</size></box></geometry></collision>
        <visual name='body'><pose>0 0 0.44 0 0 0</pose><geometry><box><size>{length} 0.7 0.88</size></box></geometry>
          <material><ambient>0.55 0.6 0.65 1</ambient><diffuse>0.55 0.6 0.65 1</diffuse></material></visual>
        <visual name='top'><pose>0 0 0.895 0 0 0</pose><geometry><box><size>{length + 0.04} 0.74 0.03</size></box></geometry>
          <material><ambient>0.95 0.95 0.93 1</ambient><diffuse>0.95 0.95 0.93 1</diffuse></material></visual>
        <visual name='microscope_base'><pose>-0.4 0.05 0.95 0 0 0</pose><geometry><box><size>0.22 0.28 0.08</size></box></geometry>
          <material><ambient>0.15 0.15 0.18 1</ambient><diffuse>0.15 0.15 0.18 1</diffuse></material></visual>
        <visual name='microscope_arm'><pose>-0.4 0.12 1.1 0.3 0 0</pose><geometry><cylinder><radius>0.035</radius><length>0.3</length></cylinder></geometry>
          <material><ambient>0.9 0.9 0.92 1</ambient><diffuse>0.9 0.9 0.92 1</diffuse></material></visual>
        <visual name='centrifuge'><pose>0.45 0.05 1.0 0 0 0</pose><geometry><cylinder><radius>0.17</radius><length>0.18</length></cylinder></geometry>
          <material><ambient>0.85 0.88 0.9 1</ambient><diffuse>0.85 0.88 0.9 1</diffuse></material></visual>
        <visual name='rack'><pose>0.0 -0.1 0.96 0 0 0</pose><geometry><box><size>0.3 0.12 0.1</size></box></geometry>
          <material><ambient>0.2 0.45 0.85 1</ambient><diffuse>0.2 0.45 0.85 1</diffuse></material></visual>
      </link>
    </model>"""


def include_sdf(name, model, x, y, yaw, z=0.0):
    return (f"    <include>\n      <uri>model://{model}</uri>\n      <name>{name}</name>\n"
            f"      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>\n      <static>true</static>\n    </include>")


def light_sdf(i, x, y):
    return f"""
    <light type='point' name='v3_ceiling_light_{i}'>
      <pose>{x} {y} 2.9 0 0 0</pose>
      <diffuse>0.55 0.55 0.52 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>9</range><constant>0.4</constant><linear>0.08</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>"""


def sign_texture(text, direction):
    os.makedirs(SIGN_DIR, exist_ok=True)
    fname = f"{text.lower().replace(' ', '_')}_{direction.lower()}.png"
    path = os.path.join(SIGN_DIR, fname)
    blue = (20, 50, 190)
    img = Image.new("RGB", (640, 320), (244, 244, 244))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([4, 4, 635, 315], radius=14, outline=blue, width=16)
    d.rectangle([22, 22, 617, 297], outline=(200, 200, 200), width=3)
    size = 64 if len(text) <= 10 else 46
    font = ImageFont.truetype(FONT, size)
    tx = 52 if direction != "LEFT" else 190
    d.text((tx, 160), text, font=font, fill=blue, anchor="lm")
    ay = 160
    if direction == "RIGHT":
        d.line([(480, ay), (570, ay)], fill=blue, width=18)
        d.polygon([(585, ay), (550, ay - 28), (550, ay + 28)], fill=blue)
    elif direction == "LEFT":
        d.line([(70, ay), (160, ay)], fill=blue, width=18)
        d.polygon([(55, ay), (90, ay - 28), (90, ay + 28)], fill=blue)
    else:  # STRAIGHT: up arrow at the right
        d.line([(545, 230), (545, 110)], fill=blue, width=18)
        d.polygon([(545, 85), (515, 125), (575, 125)], fill=blue)
    img.save(path)
    return fname, path


def board_visual(texture, pose="0 0 0 0 0 0"):
    return f"""<visual name='board_visual'>
          <pose>{pose}</pose>
          <geometry><box><size>0.8 0.03 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular>
            <emissive>0.08 0.08 0.08 1</emissive>
            <pbr><metal><albedo_map>{texture}</albedo_map><roughness>0.2</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>"""


def sign_sdf(sid, x, y, z, yaw, texture, post):
    """post=True: free-standing / projecting post sign (thin pole collision + pole visual at the wall side)."""
    pole = ""
    if post:
        pole = f"""<collision name='pole'><pose>0 0 {-z / 2:.3f} 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>{z:.3f}</length></cylinder></geometry></collision>
        <visual name='pole'><pose>0 0 {-z / 2:.3f} 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>{z:.3f}</length></cylinder></geometry>
          <material><ambient>0.6 0.6 0.62 1</ambient><diffuse>0.6 0.6 0.62 1</diffuse></material></visual>"""
    return f"""
    <model name='{sid}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name='link'>
        {board_visual(texture)}
        {pole}
      </link>
    </model>"""


def plaque_sdf(name, x, y, z, yaw, texture):
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>0.8 0.03 0.4</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient>
            <pbr><metal><albedo_map>{texture}</albedo_map><metalness>0</metalness><roughness>0.8</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def payload_sdf(x, y, z):
    return f"""
    <model name='v3_payload_blood_sample_carrier'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name='link'>
        <visual name='body'><geometry><box><size>0.26 0.18 0.14</size></box></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material></visual>
        <visual name='lid_band'><pose>0 0 0.072 0 0 0</pose><geometry><box><size>0.27 0.19 0.02</size></box></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
      </link>
    </model>"""


def dynamic_sdf(name, visual, kind, park):
    fuel, mesh, zoff = visual
    if kind == "person":
        col = "<collision name='collision'><geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry></collision>"
        mass, inertia = 70.0, "<ixx>5.0</ixx><iyy>5.0</iyy><izz>1.5</izz>"
    else:
        col = "<collision name='collision'><pose>0 0 -0.5 0 0 0</pose><geometry><box><size>0.9 0.6 0.7</size></box></geometry></collision>"
        mass, inertia = 18.0, "<ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>"
    return f"""
    <model name='{name}'>
      <pose>{park[0]:.3f} {park[1]:.3f} 0.85 0 0 1.5708</pose>
      <link name='link'>
        <inertial><mass>{mass}</mass><inertia>{inertia}</inertia></inertial>
        {col}
        <visual name='visual'><pose>0 0 {zoff} 0 0 0</pose><geometry><mesh><uri>model://{fuel}/{mesh}</uri></mesh></geometry></visual>
      </link>
    </model>"""


def additions_sdf():
    out = ["    <!-- ===== V3 additions (V3/scripts/v3_hospital_additions.py) ===== -->"]
    out += [light_sdf(i, x, y) for i, (x, y) in enumerate(LIGHTS)]
    out.append(bench_sdf("v3_lab_bench_1", 8.6, 16.45, 2.0))
    out.append(bench_sdf("v3_lab_bench_2", 6.5, 15.3, 1.6, 1.5708))
    out += [include_sdf(*p) for p in PROPS]
    out.append(payload_sdf(-7.15, -16.6, 0.86))
    tex = {"collection_point": os.path.join(MARKER_DIR, "collection_point.png"),
           "laboratory_test_point": os.path.join(MARKER_DIR, "laboratory_test_point.png")}
    out += [plaque_sdf(n, x, y, z, yaw, tex[k]) for n, x, y, z, yaw, k in L.PLAQUES]
    for sid, _j, _d, text, direction, x, y, z, yaw, _a in L.SIGNS:
        _f, path = sign_texture(text, direction)
        flush = abs(math.sin(yaw)) > 0.5  # boards facing east/west are wall-mounted in this layout
        out.append(sign_sdf(sid, x, y, z, yaw, path, post=not flush))
    out += [dynamic_sdf(n, vis, kind, park) for n, vis, kind, _line, park, _v in L.TRAFFIC]
    return "\n".join(out)
