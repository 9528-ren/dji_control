#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 生成 Pilot 2 控件面板配置 widget_cfg/widget_config.json
# 控件 index 必须与 module/widget/WidgetHandler.h 中的枚举一致
# 图标由 script/gen_icons.py 生成, 请先执行 gen_icons.py
#
# 注意: Pilot 2 大屏面板为每行 2 列网格, 按 widget_list 顺序从左到右、
# 从上到下排布; list 控件会展开成多行选项打乱网格, 因此全部使用 button.
#
# 用法: python3 script/gen_widget_cfg.py [PSDK根目录]
import json
import os
import shutil
import sys

PSDK_ROOT = sys.argv[1] if len(sys.argv) > 1 else "/home/dji/Payload-SDK-3.17.0"
SRC = os.path.join(PSDK_ROOT, "samples/sample_c/module_sample/widget/widget_file/cn_big_screen")
DST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "widget_cfg")

# 官方底图: 保证目录自洽
COPY_ICONS = ["icon_switch_select.png", "icon_switch_unselect.png"]


def button(index, name, icon):
    return {
        "widget_index": index, "widget_type": "button", "widget_name": name,
        "icon_file_set": {"icon_file_name_selected": icon,
                          "icon_file_name_unselected": icon},
        "customize_rc_buttons_config": {"is_enable": True,
                                        "mapping_config_display_order": index},
    }


os.makedirs(DST, exist_ok=True)
for icon in COPY_ICONS:
    src = os.path.join(SRC, icon)
    if os.path.exists(src):
        shutil.copy(src, DST)

d = json.load(open(os.path.join(SRC, "widget_config.json")))
# 配置内容版本号: Pilot 2 按此字段判断是否重新下载控件配置, 每次改动必须递增
d["version"] = {"major": 5, "minor": 0}
# 喊话器控件: 语音模块已移出项目(平台不支持), 保留 speaker 字段会让 Pilot 2
# 显示喊话器入口但点击无响应, 故显式移除
d["main_interface"].pop("speaker", None)
d["config_interface"].pop("text_input_box", None)
# 浮窗用于显示飞行状态提示
d["main_interface"]["floating_window"] = {"is_enable": True}
d["main_interface"]["widget_list"] = [
    # 行1: 起飞 | 降落
    button(0, "起飞", "icon_takeoff.png"),
    button(1, "降落", "icon_landing.png"),
    # 行2: 上仰 | 下俯
    button(2, "上仰", "icon_gimbal_up.png"),
    button(3, "下俯", "icon_gimbal_down.png"),
    # 行3: 回中 | 飞往目标
    button(4, "回中", "icon_gimbal_recenter.png"),
    button(5, "飞往目标", "icon_fly_target.png"),
    # 行4: 前进 | 后退
    button(6, "前进", "icon_move_forward.png"),
    button(7, "后退", "icon_move_backward.png"),
    # 行5: 左移 | 右移
    button(8, "左移", "icon_move_left.png"),
    button(9, "右移", "icon_move_right.png"),
    # 行6: 返航 | RTK返航
    button(10, "返航", "icon_go_home.png"),
    button(11, "RTK返航", "icon_go_home_rtk.png"),
]
d["config_interface"]["widget_list"] = []

out = os.path.join(DST, "widget_config.json")
json.dump(d, open(out, "w"), ensure_ascii=False, indent=2)
print("widget config written: %s" % os.path.normpath(out))
ws = d["main_interface"]["widget_list"]
for i in range(0, len(ws), 2):
    row = " | ".join("[%d] %s" % (w["widget_index"], w["widget_name"])
                     for w in ws[i:i + 2])
    print("  row%d: %s" % (i // 2 + 1, row))
