# RoboMaster Fixed Maze Mission

โปรเจกต์ภารกิจ Pickup → A* → Drop → Exit สำหรับ RoboMaster EP แยกเป็นโมดูลเพื่อให้แก้และทดสอบง่ายขึ้น โดยพฤติกรรมการควบคุมรถยังมาจากโค้ดเดิม

## รันโปรแกรม

```powershell
cd robomaster_fixed_maze_mission
py -m pip install -r requirements.txt
py .\main.py
```

โหมดเดิมแบบไม่มี GUI:

```powershell
py .\main.py --legacy
```

## ไฟล์ผลลัพธ์หลังจบภารกิจ

- `*_run_YYYYMMDD_HHMMSS.json` ข้อมูลตั้งค่า แผนที่ หลักฐานกำแพง และประวัติเซนเซอร์
- `*_map.svg` แผนที่สุดท้ายและเส้นทางที่วิ่งจริง
- `*_sensor_graph.svg` กราฟระยะ Front ToF, Left Sharp และ Right Sharp ในแต่ละจุดสแกน

ไฟล์ถูกสร้างในโฟลเดอร์ที่ใช้รัน `main.py`

## เหตุผลที่กำแพงสีแดงเคยหาย

โค้ดเดิมลบ `sensor_walls` ทันทีเมื่อการสแกนครั้งหลังอ่านว่าขอบเดิมเปิด การอ่าน Sharp ตอนรถเยื้องศูนย์หรือมองผ่านมุมกำแพงจึงลบหลักฐานเดิมได้ เวอร์ชันนี้วาดกำแพงเมื่อพบได้ทันที แต่ต้องอ่านว่าเปิดต่อเนื่อง 3 รอบจึงลบ หรือจะลบทันทีเมื่อรถข้ามขอบนั้นจริง

## โครงสร้าง

```text
robomaster_fixed_maze_mission/
├── main.py
├── requirements.txt
├── robomaster_mission/
│   ├── configuration.py   # ค่าตั้งและ validation
│   ├── grid_map.py        # แผนที่และหลักฐานกำแพง
│   ├── planning.py        # A* และ topological graph
│   ├── mission.py         # sensor, motion, pickup/drop, navigation
│   ├── reporting.py       # JSON, map SVG, sensor graph SVG
│   ├── version.py         # เวอร์ชันของรูปแบบผลลัพธ์
│   └── gui.py             # Tkinter GUI
└── tests/
    └── test_wall_evidence.py
```

## ทดสอบเฉพาะส่วนที่ไม่ต้องต่อรถ

```powershell
py -m unittest discover -s tests -v
```
