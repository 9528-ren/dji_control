#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 生成 Pilot 2 控件图标(96x96 PNG, 与 PSDK 官方示例尺寸一致)
# 用法: python3 script/gen_icons.py [输出目录]
# 图标风格: 深色圆角底 + 白色描边图形, 列表项用彩色底区分
import os
import sys

from PIL import Image, ImageDraw

SIZE = 96
BG = (30, 34, 42, 255)          # 按钮底色
FG = (255, 255, 255, 255)       # 前景白
ACC_GIMBAL = (46, 160, 67, 255)  # 云台列表项: 绿
ACC_TAKE = (32, 120, 200, 255)   # 起飞: 蓝
ACC_LAND = (200, 120, 32, 255)   # 降落: 橙
ACC_HOME = (150, 60, 180, 255)   # 返航: 紫
ACC_TARGET = (200, 60, 60, 255)  # 飞往目标: 红


def new_img(bg=BG):
    img = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([2, 2, SIZE - 3, SIZE - 3], radius=20, fill=bg)
    return img, d


def save(img, out_dir, name):
    path = os.path.join(out_dir, name)
    img.save(path)
    print("icon written: %s" % path)


def arrow(deg, color=FG, width=10):
    """朝上的箭头, 再按 deg 顺时针旋转."""
    img, d = new_img()
    cx, cy = SIZE // 2, SIZE // 2
    # 箭杆 + 箭头(未旋转时指向 -y 即"上")
    d.line([(cx, cy + 26), (cx, cy - 22)], fill=color, width=width)
    d.polygon([(cx, cy - 34), (cx - 18, cy - 12), (cx + 18, cy - 12)], fill=color)
    return img.rotate(deg, resample=Image.BICUBIC, expand=False)


def make_move_icons(out):
    # 前进=朝上(机头方向), 后退=朝下, 左移=朝左, 右移=朝右
    save(arrow(0), out, "icon_move_forward.png")
    save(arrow(180), out, "icon_move_backward.png")
    save(arrow(270), out, "icon_move_left.png")
    save(arrow(90), out, "icon_move_right.png")


def make_takeoff(out):
    img, d = new_img(ACC_TAKE)
    cx, cy = SIZE // 2, SIZE // 2 + 6
    # 机身 + 两臂 + 旋翼 + 上升箭头
    d.ellipse([cx - 12, cy - 6, cx + 12, cy + 10], fill=FG)
    d.line([(cx - 26, cy - 12), (cx - 10, cy)], fill=FG, width=6)
    d.line([(cx + 26, cy - 12), (cx + 10, cy)], fill=FG, width=6)
    d.line([(cx - 36, cy - 16), (cx - 16, cy - 16)], fill=FG, width=5)
    d.line([(cx + 16, cy - 16), (cx + 36, cy - 16)], fill=FG, width=5)
    d.polygon([(cx, cy - 44), (cx - 12, cy - 26), (cx + 12, cy - 26)], fill=FG)
    save(img, out, "icon_takeoff.png")


def make_landing(out):
    img, d = new_img(ACC_LAND)
    cx, cy = SIZE // 2, SIZE // 2 - 6
    d.ellipse([cx - 12, cy - 10, cx + 12, cy + 6], fill=FG)
    d.line([(cx - 26, cy - 16), (cx - 10, cy - 4)], fill=FG, width=6)
    d.line([(cx + 26, cy - 16), (cx + 10, cy - 4)], fill=FG, width=6)
    d.line([(cx - 36, cy - 20), (cx - 16, cy - 20)], fill=FG, width=5)
    d.line([(cx + 16, cy - 20), (cx + 36, cy - 20)], fill=FG, width=5)
    d.polygon([(cx, cy + 34), (cx - 12, cy + 16), (cx + 12, cy + 16)], fill=FG)
    save(img, out, "icon_landing.png")


def make_target(out):
    img, d = new_img(ACC_TARGET)
    cx, cy = SIZE // 2, SIZE // 2
    for r in (32, 20):
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=FG, width=6)
    d.ellipse([cx - 7, cy - 7, cx + 7, cy + 7], fill=FG)
    save(img, out, "icon_fly_target.png")


def make_go_home(out):
    img, d = new_img(ACC_HOME)
    cx, cy = SIZE // 2, SIZE // 2
    # 房子
    d.polygon([(cx - 30, cy + 2), (cx, cy - 26), (cx + 30, cy + 2)], fill=FG)
    d.rectangle([cx - 22, cy + 2, cx + 22, cy + 30], fill=FG)
    d.rectangle([cx - 7, cy + 12, cx + 7, cy + 30], fill=BG)
    save(img, out, "icon_go_home.png")


def make_go_home_rtk(out):
    # RTK 返航: 房子 + 顶部卫星信号弧
    bg = (24, 100, 160, 255)
    img, d = new_img(bg)
    cx, cy = SIZE // 2, SIZE // 2 + 10
    d.polygon([(cx - 24, cy), (cx, cy - 20), (cx + 24, cy)], fill=FG)
    d.rectangle([cx - 18, cy, cx + 18, cy + 22], fill=FG)
    d.rectangle([cx - 6, cy + 8, cx + 6, cy + 22], fill=bg)
    # 卫星信号弧(两道)
    for r in (14, 24):
        d.arc([cx - r, 12 - r, cx + r, 12 + r], start=200, end=340,
              fill=FG, width=5)
    d.ellipse([cx - 4, 6, cx + 4, 14], fill=FG)
    save(img, out, "icon_go_home_rtk.png")


def make_gimbal_list(out):
    # 列表控件本身的图标: 云台镜头
    img, d = new_img()
    cx, cy = SIZE // 2, SIZE // 2
    d.rounded_rectangle([cx - 26, cy - 22, cx + 26, cy + 22], radius=8,
                        outline=FG, width=6)
    d.ellipse([cx - 13, cy - 13, cx + 13, cy + 13], outline=FG, width=6)
    d.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=FG)
    save(img, out, "icon_gimbal.png")

    # 列表三项: 上仰 / 下俯 / 回中
    img, d = new_img(ACC_GIMBAL)
    d.pieslice([24, 30, 72, 78], start=200, end=340, fill=FG)
    d.rectangle([44, 40, 52, 54], fill=FG)
    save(img, out, "icon_gimbal_up.png")

    img, d = new_img(ACC_GIMBAL)
    d.pieslice([24, 18, 72, 66], start=20, end=160, fill=FG)
    d.rectangle([44, 42, 52, 56], fill=FG)
    save(img, out, "icon_gimbal_down.png")

    img, d = new_img(ACC_GIMBAL)
    d.arc([26, 26, 70, 70], start=40, end=320, fill=FG, width=7)
    d.polygon([(70, 26), (56, 24), (66, 40)], fill=FG)
    d.ellipse([42, 42, 54, 54], fill=FG)
    save(img, out, "icon_gimbal_recenter.png")


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "widget_cfg")
    out = os.path.normpath(out)
    os.makedirs(out, exist_ok=True)
    make_move_icons(out)
    make_takeoff(out)
    make_landing(out)
    make_target(out)
    make_go_home(out)
    make_go_home_rtk(out)
    make_gimbal_list(out)
    print("all icons generated in %s" % out)


if __name__ == "__main__":
    main()
