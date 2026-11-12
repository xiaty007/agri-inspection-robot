from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


OUTPUT = r"C:\Users\33615\Documents\New project111\二维雷达建图启动操作手册.docx"


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=100, start=120, bottom=100, end=120):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for tag, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{tag}"))
        if node is None:
            node = OxmlElement(f"w:{tag}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_table_borders(table, color="D9D9D9", size="6"):
    tbl_pr = table._tbl.tblPr
    borders = tbl_pr.find(qn("w:tblBorders"))
    if borders is None:
        borders = OxmlElement("w:tblBorders")
        tbl_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        element = borders.find(qn(f"w:{edge}"))
        if element is None:
            element = OxmlElement(f"w:{edge}")
            borders.append(element)
        element.set(qn("w:val"), "single")
        element.set(qn("w:sz"), size)
        element.set(qn("w:space"), "0")
        element.set(qn("w:color"), color)


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    tr_pr.append(tbl_header)


def set_run_font(run, name="Microsoft YaHei", size=10.5, bold=False, color="000000"):
    run.font.name = name
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = RGBColor.from_string(color)
    r_pr = run._element.get_or_add_rPr()
    r_fonts = r_pr.rFonts
    if r_fonts is None:
        r_fonts = OxmlElement("w:rFonts")
        r_pr.insert(0, r_fonts)
    for key in ("ascii", "hAnsi", "eastAsia", "cs"):
        r_fonts.set(qn(f"w:{key}"), name)


def set_paragraph_keep(paragraph, keep_next=False, keep_lines=True):
    p_pr = paragraph._p.get_or_add_pPr()
    if keep_next:
        node = OxmlElement("w:keepNext")
        p_pr.append(node)
    if keep_lines:
        node = OxmlElement("w:keepLines")
        p_pr.append(node)


def add_body(doc, text, bold_lead=None):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.line_spacing = 1.25
    if bold_lead and text.startswith(bold_lead):
        set_run_font(p.add_run(bold_lead), bold=True)
        set_run_font(p.add_run(text[len(bold_lead):]))
    else:
        set_run_font(p.add_run(text))
    return p


def add_bullet(doc, text, level=0):
    style = "List Bullet" if level == 0 else "List Bullet 2"
    p = doc.add_paragraph(style=style)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.line_spacing = 1.15
    set_run_font(p.add_run(text))
    return p


def add_number(doc, text):
    global number_counter
    number_counter += 1
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.75)
    p.paragraph_format.first_line_indent = Cm(-0.55)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.line_spacing = 1.15
    set_run_font(p.add_run(f"{number_counter}.  {text}"))
    return p


def add_code(doc, code):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.7)
    p.paragraph_format.right_indent = Cm(0.4)
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(7)
    p.paragraph_format.line_spacing = 1.0
    p_pr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "F3F5F7")
    p_pr.append(shd)
    for index, line in enumerate(code.splitlines()):
        run = p.add_run(line)
        set_run_font(run, name="Consolas", size=8.6, color="202124")
        if index < len(code.splitlines()) - 1:
            run.add_break()
    set_paragraph_keep(p, keep_lines=True)
    return p


def add_heading(doc, text, level=1):
    p = doc.add_paragraph(text, style=f"Heading {level}")
    p.paragraph_format.space_before = Pt(14 if level == 1 else 9)
    p.paragraph_format.space_after = Pt(6)
    set_paragraph_keep(p, keep_next=True)
    for run in p.runs:
        set_run_font(run, size=15 if level == 1 else 12, bold=True)
    return p


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run("第 ")
    set_run_font(run, size=8.5, color="666666")
    fld_char_1 = OxmlElement("w:fldChar")
    fld_char_1.set(qn("w:fldCharType"), "begin")
    instr_text = OxmlElement("w:instrText")
    instr_text.set(qn("xml:space"), "preserve")
    instr_text.text = "PAGE"
    fld_char_2 = OxmlElement("w:fldChar")
    fld_char_2.set(qn("w:fldCharType"), "end")
    run._r.append(fld_char_1)
    run._r.append(instr_text)
    run._r.append(fld_char_2)
    run2 = paragraph.add_run(" 页")
    set_run_font(run2, size=8.5, color="666666")


def add_info_table(doc):
    data = [
        ("树莓派", "Raspberry Pi 5B 8GB，Ubuntu 24.04，ROS 2 Jazzy"),
        ("STM32 和 PNI", "/dev/ttyUSB0"),
        ("思岚 C1M1 雷达", "/dev/ttyUSB1"),
        ("雷达话题", "/scan"),
        ("目标 TF", "map 到 odom 到 base_link 到 laser"),
    ]
    table = doc.add_table(rows=1, cols=2)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.columns[0].width = Cm(4.2)
    table.columns[1].width = Cm(11.5)
    set_table_borders(table)
    hdr = table.rows[0].cells
    hdr[0].text = "项目"
    hdr[1].text = "当前配置"
    set_repeat_table_header(table.rows[0])
    for cell in hdr:
        set_cell_shading(cell, "1F4E78")
        set_cell_margins(cell)
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        for p in cell.paragraphs:
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            for run in p.runs:
                set_run_font(run, size=9.5, bold=True, color="FFFFFF")
    for idx, (key, value) in enumerate(data):
        cells = table.add_row().cells
        cells[0].text = key
        cells[1].text = value
        if idx % 2 == 1:
            set_cell_shading(cells[0], "F5F8FB")
            set_cell_shading(cells[1], "F5F8FB")
        for col, cell in enumerate(cells):
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            for p in cell.paragraphs:
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER if col == 0 else WD_ALIGN_PARAGRAPH.LEFT
                for run in p.runs:
                    set_run_font(run, size=9.3, bold=(col == 0))
    doc.add_paragraph().paragraph_format.space_after = Pt(0)


doc = Document()
number_counter = 0
section = doc.sections[0]
section.top_margin = Cm(2.0)
section.bottom_margin = Cm(1.8)
section.left_margin = Cm(2.2)
section.right_margin = Cm(2.2)
section.header_distance = Cm(0.8)
section.footer_distance = Cm(0.8)

styles = doc.styles
normal = styles["Normal"]
normal.font.name = "Microsoft YaHei"
normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
normal.font.size = Pt(10.5)
for style_name in ("Title", "Heading 1", "Heading 2"):
    style = styles[style_name]
    style.font.name = "Microsoft YaHei"
    style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    style.font.color.rgb = RGBColor(0, 0, 0)

title_style_ppr = styles["Title"]._element.get_or_add_pPr()
title_style_border = title_style_ppr.find(qn("w:pBdr"))
if title_style_border is not None:
    title_style_ppr.remove(title_style_border)

title = doc.add_paragraph(style="Title")
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
title.paragraph_format.space_before = Pt(16)
title.paragraph_format.space_after = Pt(10)
set_run_font(title.add_run("二维雷达建图启动操作手册"), size=24, bold=True)

subtitle = doc.add_paragraph()
subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
subtitle.paragraph_format.space_after = Pt(20)
set_run_font(subtitle.add_run("ROS 2 Jazzy  思岚 C1M1  Raspberry Pi 5B"), size=11, color="4F4F4F")

intro = doc.add_paragraph()
intro.paragraph_format.space_after = Pt(12)
intro.paragraph_format.line_spacing = 1.3
set_run_font(intro.add_run(
    "本手册记录每次树莓派开机后启动二维雷达建图的命令和顺序。正式开发优先使用 RF2O 与 SLAM Toolbox，Cartographer 2D 作为纯雷达对照方案。两套方案不能同时运行。"
), size=11)

add_info_table(doc)

add_heading(doc, "使用原则", 1)
add_bullet(doc, "主方案启动顺序为雷达 固定 TF RF2O SLAM Toolbox RViz 保存地图")
add_bullet(doc, "RF2O 运行时不得由 PNI 或其他节点重复发布 odom 到 base_link")
add_bullet(doc, "Cartographer 运行时必须停止 RF2O SLAM Toolbox 和 PNI 动态 TF")
add_bullet(doc, "树莓派重启或重新插拔设备后，USB0 和 USB1 可能互换")

doc.add_page_break()

add_heading(doc, "1 启动前检查", 1)
add_body(doc, "先确认两个USB串口设备。推荐同时查看稳定的 by id 名称，避免设备编号互换后打开错误串口。")
add_code(doc, "ls -l /dev/ttyUSB*\nls -l /dev/serial/by-id/")

add_body(doc, "当前约定如下。")
add_code(doc, "STM32 和 PNI    /dev/ttyUSB0\n思岚 C1M1 雷达  /dev/ttyUSB1")

add_heading(doc, "2 每个终端的环境初始化", 1)
add_body(doc, "每打开一个新终端，都先加载ROS 2和工作区环境。")
add_code(doc, "source /opt/ros/jazzy/setup.bash\nsource ~/ros2_ws/install/setup.bash")

add_heading(doc, "3 主方案 RF2O 与 SLAM Toolbox", 1)
add_body(doc, "该方案使用RF2O从连续雷达帧计算短期里程计，再由SLAM Toolbox完成扫描匹配、位姿图建图和回环优化。")

add_heading(doc, "3 1 终端一启动雷达", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\nsource ~/ros2_ws/install/setup.bash\n\nros2 run rplidar_ros rplidar_composition --ros-args \\\n  -p serial_port:=/dev/ttyUSB1 \\\n  -p serial_baudrate:=460800 \\\n  -p frame_id:=laser \\\n  -p angle_compensate:=true \\\n  -p scan_mode:=Standard")
add_body(doc, "另开终端检查雷达话题。命令应持续显示频率。")
add_code(doc, "ros2 topic hz /scan")

add_heading(doc, "3 2 终端二发布雷达固定 TF", 2)
add_body(doc, "如果现有雷达启动文件已经发布 base_link 到 laser，则不要重复运行本命令。当前测试使用零偏移，后续应改为雷达相对车体中心的实测安装位置。")
add_code(doc, "source /opt/ros/jazzy/setup.bash\n\nros2 run tf2_ros static_transform_publisher \\\n  --x 0 --y 0 --z 0 \\\n  --roll 0 --pitch 0 --yaw 0 \\\n  --frame-id base_link \\\n  --child-frame-id laser")
add_code(doc, "ros2 run tf2_ros tf2_echo base_link laser")

add_heading(doc, "3 3 终端三启动 RF2O", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\nsource ~/ros2_ws/install/setup.bash\n\nros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py")
add_body(doc, "RF2O默认订阅 /scan，发布 /odom_rf2o，并发布 odom 到 base_link。缓慢移动小车时，位置和旋转应持续变化。")
add_code(doc, "ros2 topic hz /odom_rf2o\nros2 topic echo /odom_rf2o --once\nros2 run tf2_ros tf2_echo odom base_link")
add_body(doc, "终端偶尔显示 Waiting for laser_scans 不一定表示节点失效。只要 /odom_rf2o 持续发布并随小车运动变化，就可以继续。", bold_lead="终端偶尔显示")

add_heading(doc, "3 4 终端四启动 SLAM Toolbox", 2)
add_body(doc, "先定位之前修改过的参数文件。")
add_code(doc, "find ~/ros2_ws -type f \\( -name \"*.yaml\" -o -name \"*.yml\" \\) | grep -E \"mapper|slam\"")
add_body(doc, "参数文件很可能位于以下路径。先用 ls 确认；如果实际路径不同，替换后续命令中的文件路径。")
add_code(doc, "ls -l ~/ros2_ws/src/slam_toolbox/config/mapper_params_online_async.yaml")
add_code(doc, "source /opt/ros/jazzy/setup.bash\nsource ~/ros2_ws/install/setup.bash\n\nros2 launch slam_toolbox online_async_launch.py \\\n  use_sim_time:=false \\\n  slam_params_file:=/home/rsp/ros2_ws/src/slam_toolbox/config/mapper_params_online_async.yaml")

add_body(doc, "参数文件应至少保持以下关键配置。")
add_code(doc, "odom_frame: odom\nmap_frame: map\nbase_frame: base_link\nscan_topic: /scan\nuse_scan_matching: true\nminimum_travel_distance: 0.05\nminimum_travel_heading: 0.087\nmap_update_interval: 1.0\nenable_interactive_mode: false")

add_heading(doc, "3 5 终端五启动 RViz", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\nrviz2")
add_body(doc, "在RViz中完成以下设置。")
add_bullet(doc, "Fixed Frame 设置为 map")
add_bullet(doc, "添加 Map 并选择 /map")
add_bullet(doc, "添加 LaserScan 并选择 /scan")
add_bullet(doc, "添加 TF 检查坐标系")

add_heading(doc, "4 TF 发布关系", 1)
add_body(doc, "正常的坐标关系如下。每一条动态TF必须只有一个发布者。")
add_code(doc, "map -> odom -> base_link -> laser")

table = doc.add_table(rows=1, cols=3)
table.alignment = WD_TABLE_ALIGNMENT.CENTER
table.autofit = False
widths = [Cm(5.0), Cm(5.0), Cm(5.5)]
for idx, width in enumerate(widths):
    table.columns[idx].width = width
set_table_borders(table)
headers = ["TF关系", "主方案发布者", "Cartographer方案发布者"]
for idx, text in enumerate(headers):
    cell = table.rows[0].cells[idx]
    cell.text = text
    set_cell_shading(cell, "1F4E78")
    set_cell_margins(cell)
    cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
    for p in cell.paragraphs:
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        for run in p.runs:
            set_run_font(run, size=9.0, bold=True, color="FFFFFF")
set_repeat_table_header(table.rows[0])
rows = [
    ("map 到 odom", "SLAM Toolbox", "Cartographer"),
    ("odom 到 base_link", "RF2O", "Cartographer"),
    ("base_link 到 laser", "静态 TF", "静态 TF"),
]
for ridx, row_data in enumerate(rows):
    cells = table.add_row().cells
    for cidx, value in enumerate(row_data):
        cells[cidx].text = value
        set_cell_margins(cells[cidx])
        cells[cidx].vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        if ridx % 2 == 1:
            set_cell_shading(cells[cidx], "F5F8FB")
        for p in cells[cidx].paragraphs:
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            for run in p.runs:
                set_run_font(run, size=9.2)

add_heading(doc, "5 主方案运行检查", 1)
add_code(doc, "ros2 topic hz /scan\nros2 topic hz /odom_rf2o\nros2 topic hz /map\nros2 run tf2_ros tf2_echo map odom\nros2 run tf2_ros tf2_echo odom base_link\nros2 run tf2_ros tf2_echo base_link laser")
add_body(doc, "需要查看整棵TF树时执行。")
add_code(doc, "ros2 run tf2_tools view_frames")

add_heading(doc, "6 保存地图", 1)
add_body(doc, "先创建地图目录，再保存Nav2可直接加载的栅格地图。")
add_code(doc, "mkdir -p ~/maps\n\nros2 run nav2_map_server map_saver_cli \\\n  -f /home/rsp/maps/c1m1_map")
add_body(doc, "命令会生成 c1m1_map.pgm 和 c1m1_map.yaml。若要以后继续建图，还应保存SLAM Toolbox完整位姿图。")
add_code(doc, "ros2 service call /slam_toolbox/serialize_map \\\n  slam_toolbox/srv/SerializePoseGraph \\\n  \"{filename: '/home/rsp/maps/c1m1_posegraph'}\"")

doc.add_page_break()

add_heading(doc, "7 Cartographer 纯雷达对照方案", 1)
add_body(doc, "切换前，在原有终端中按 Ctrl C，停止 RF2O、SLAM Toolbox、PNI动态TF和任何静态 odom 到 base_link。保留雷达驱动与 base_link 到 laser 固定TF。")

add_heading(doc, "7 1 终端一启动雷达", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\nsource ~/ros2_ws/install/setup.bash\n\nros2 run rplidar_ros rplidar_composition --ros-args \\\n  -p serial_port:=/dev/ttyUSB1 \\\n  -p serial_baudrate:=460800 \\\n  -p frame_id:=laser \\\n  -p angle_compensate:=true \\\n  -p scan_mode:=Standard")

add_heading(doc, "7 2 终端二发布固定 TF", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\n\nros2 run tf2_ros static_transform_publisher \\\n  --x 0 --y 0 --z 0 \\\n  --roll 0 --pitch 0 --yaw 0 \\\n  --frame-id base_link \\\n  --child-frame-id laser")

add_heading(doc, "7 3 终端三启动 Cartographer", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\n\nros2 run cartographer_ros cartographer_node \\\n  -configuration_directory /home/rsp/ros2_ws/cartographer_config \\\n  -configuration_basename c1m1_2d.lua \\\n  --ros-args -p use_sim_time:=false")

add_heading(doc, "7 4 终端四发布占据栅格地图", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\n\nros2 run cartographer_ros cartographer_occupancy_grid_node \\\n  -resolution 0.05 \\\n  -publish_period_sec 0.5")

add_heading(doc, "7 5 终端五启动 RViz", 2)
add_code(doc, "source /opt/ros/jazzy/setup.bash\nrviz2")
add_body(doc, "RViz仍将Fixed Frame设为map，并添加 /map、/scan 和TF。")

doc.add_page_break()

add_heading(doc, "8 常见问题检查", 1)

add_heading(doc, "8 1 RViz没有地图", 2)
add_number(doc, "确认 /scan 有持续数据。")
add_number(doc, "确认 base_link 到 laser 能被 tf2_echo 查询。")
add_number(doc, "确认主方案的 /odom_rf2o 正在发布。")
add_number(doc, "确认RViz的Fixed Frame为map。")
add_number(doc, "检查SLAM节点终端是否出现消息过滤或TF超时。")

add_heading(doc, "8 2 地图不随运动更新", 2)
add_number(doc, "缓慢移动或旋转小车，并观察 /odom_rf2o 的x、y和yaw是否变化。")
add_number(doc, "确认没有第二个节点重复发布 odom 到 base_link。")
add_number(doc, "确认SLAM Toolbox参数中的最小移动距离和最小转角不是过大的数值。")
add_number(doc, "避免只在空旷、对称或缺少墙角特征的环境中测试。")

add_heading(doc, "8 3 地图重影或乱飘", 2)
add_number(doc, "降低小车旋转和移动速度。")
add_number(doc, "检查雷达是否固定牢靠，雷达坐标方向是否正确。")
add_number(doc, "检查 /scan 时间戳和发布频率是否稳定。")
add_number(doc, "检查TF树中是否存在重复发布者。")
add_number(doc, "PNI加速度积分不能作为长期可靠的位置里程计。")

add_heading(doc, "9 建议的后续结构", 1)
add_body(doc, "正式开发继续使用SLAM Toolbox。加入左右轮编码器后，由编码器提供平移和短期里程计，PNI只提供姿态、航向或角速度，再用robot_localization融合。")
add_code(doc, "轮速编码器 + PNI\n        |\nrobot_localization\n        |\nodom -> base_link\n        |\nSLAM Toolbox + Nav2")
add_body(doc, "RF2O届时可保留为辅助里程计或对照数据。Cartographer继续用于纯雷达测试，不与主方案同时启动。")

footer = section.footer
footer_p = footer.paragraphs[0]
add_page_number(footer_p)

doc.core_properties.title = "二维雷达建图启动操作手册"
doc.core_properties.subject = "ROS 2 Jazzy下RF2O SLAM Toolbox和Cartographer的启动顺序"
doc.core_properties.keywords = "ROS2, SLAM Toolbox, RF2O, Cartographer, RPLIDAR"
doc.core_properties.comments = ""

doc.save(OUTPUT)
print(OUTPUT)
