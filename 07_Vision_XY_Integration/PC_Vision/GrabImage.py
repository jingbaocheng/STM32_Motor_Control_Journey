# -- coding: utf-8 --
import os
import sys
import threading
import msvcrt
import struct
import csv
import time
from datetime import datetime
import numpy as np
import cv2

print("Python executable:", sys.executable)
print("Python version:", sys.version)
print("Python bits:", struct.calcsize("P") * 8)
from ctypes import *

sys.path.append(os.path.join(os.path.dirname(__file__), "MvImport"))
from MvCameraControl_class import *


def detect_black_dot(frame):
    """
    Keep the current frozen detection pipeline:
        fixed ROI
        -> grayscale
        -> GaussianBlur
        -> THRESH_BINARY_INV + Otsu
        -> contour filtering
        -> local grayscale darkness-weighted centroid

    Return:
        result_frame, info_dict, binary

    info_dict is always returned so failed detections are still logged.
    """
    h, w = frame.shape[:2]

    # Fixed ROI: keep exactly the current project range.
    x1 = int(w * 0.25)
    x2 = int(w * 0.65)
    y1 = int(h * 0.30)
    y2 = int(h * 0.70)

    roi = frame[y1:y2, x1:x2]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Otsu threshold value is useful experimental metadata.
    otsu_value, binary = cv2.threshold(
        gray,
        0,
        255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    white_ratio = (
        float(np.count_nonzero(binary)) / float(binary.size)
        if binary.size > 0 else 0.0
    )

    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    best_contour = None
    best_score = 0.0
    best_area = 0.0
    best_circularity = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < 800 or area > 20000:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)

        if circularity < 0.50:
            continue

        score = area * circularity

        if score > best_score:
            best_score = score
            best_contour = cnt
            best_area = float(area)
            best_circularity = float(circularity)

    result = frame.copy()

    cv2.rectangle(
        result,
        (x1, y1),
        (x2, y2),
        (255, 0, 0),
        3
    )

    info = {
        "found": 0,
        "u": None,
        "v": None,
        "bbox_x": None,
        "bbox_y": None,
        "bbox_w": None,
        "bbox_h": None,
        "area": None,
        "circularity": None,
        "otsu": float(otsu_value),
        "white_ratio": float(white_ratio),
    }

    if best_contour is None:
        print(
            "Target not found: otsu=%.1f, contours=%d, white_ratio=%.4f"
            % (otsu_value, len(contours), white_ratio)
        )
        return result, info, binary

    bx, by, bw, bh = cv2.boundingRect(best_contour)

    bbox_x = bx + x1
    bbox_y = by + y1

    print(
        "bbox=(%d,%d,%d,%d)"
        % (bbox_x, bbox_y, bw, bh)
    )

    margin = 10

    rx1 = max(bx - margin, 0)
    ry1 = max(by - margin, 0)
    rx2 = min(bx + bw + margin, gray.shape[1])
    ry2 = min(by + bh + margin, gray.shape[0])

    dot_gray = gray[ry1:ry2, rx1:rx2].astype(np.float32)

    weights = 255.0 - dot_gray
    weights[weights < 60.0] = 0.0

    weight_sum = float(np.sum(weights))

    if weight_sum <= 0.0:
        print("Weighted center error")
        info.update({
            "bbox_x": bbox_x,
            "bbox_y": bbox_y,
            "bbox_w": bw,
            "bbox_h": bh,
            "area": best_area,
            "circularity": best_circularity,
        })
        return result, info, binary

    yy, xx = np.indices(weights.shape)

    cx_local = float(np.sum(xx * weights) / weight_sum)
    cy_local = float(np.sum(yy * weights) / weight_sum)

    cx_roi = cx_local + rx1
    cy_roi = cy_local + ry1

    u = float(cx_roi + x1)
    v = float(cy_roi + y1)

    print(
        "center=(%.2f,%.2f), otsu=%.1f, white_ratio=%.4f"
        % (u, v, otsu_value, white_ratio)
    )

    cnt_full = best_contour.copy()
    cnt_full[:, 0, 0] += x1
    cnt_full[:, 0, 1] += y1

    u_draw = int(round(u))
    v_draw = int(round(v))

    cv2.drawContours(result, [cnt_full], -1, (0, 255, 0), 3)

    cv2.circle(
        result,
        (u_draw, v_draw),
        10,
        (0, 0, 255),
        -1
    )

    cv2.line(
        result,
        (u_draw - 30, v_draw),
        (u_draw + 30, v_draw),
        (0, 0, 255),
        2
    )

    cv2.line(
        result,
        (u_draw, v_draw - 30),
        (u_draw, v_draw + 30),
        (0, 0, 255),
        2
    )

    info.update({
        "found": 1,
        "u": u,
        "v": v,
        "bbox_x": bbox_x,
        "bbox_y": bbox_y,
        "bbox_w": bw,
        "bbox_h": bh,
        "area": best_area,
        "circularity": best_circularity,
    })

    return result, info, binary


g_bExit = False

CAPTURE_LIMIT = 10
EXPERIMENT_NAME = "01_static_repeatability"


def save_png(path, image, label):
    """
    Windows/Chinese-path-safe PNG save.
    """
    ok, encoded = cv2.imencode(
        ".png",
        image,
        [cv2.IMWRITE_PNG_COMPRESSION, 1]
    )

    if not ok:
        print("%s encode failed: %s" % (label, path))
        return False

    try:
        encoded.tofile(path)
    except OSError as exc:
        print("%s save failed: %s" % (label, exc))
        return False

    return True


def write_repeatability_summary(session_dir, centers, total_count):
    summary_path = os.path.join(session_dir, "summary.txt")

    valid_count = len(centers)

    lines = [
        "Static repeatability summary",
        "============================",
        "total_captures=%d" % total_count,
        "valid_detections=%d" % valid_count,
        "detection_rate=%.1f%%" % (
            100.0 * valid_count / total_count if total_count else 0.0
        ),
    ]

    if valid_count >= 1:
        a = np.asarray(centers, dtype=np.float64)
        mean = a.mean(axis=0)
        rng = a.max(axis=0) - a.min(axis=0)

        lines.append("mean_u=%.4f" % mean[0])
        lines.append("mean_v=%.4f" % mean[1])
        lines.append("range_u=%.4f" % rng[0])
        lines.append("range_v=%.4f" % rng[1])

        if valid_count >= 2:
            std = a.std(axis=0, ddof=1)
            lines.append("std_u=%.4f" % std[0])
            lines.append("std_v=%.4f" % std[1])
        else:
            lines.append("std_u=NA")
            lines.append("std_v=NA")

    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))
    print("summary saved:", summary_path)


# Camera acquisition thread.
def work_thread(cam=0, pData=0, nDataSize=0):
    global g_bExit

    base_dir = os.path.dirname(os.path.abspath(__file__))
    session_tag = datetime.now().strftime("%Y%m%d_%H%M%S")

    session_dir = os.path.join(
        base_dir,
        "experiments",
        EXPERIMENT_NAME,
        session_tag
    )

    raw_dir = os.path.join(session_dir, "raw")
    result_dir = os.path.join(session_dir, "result")
    binary_dir = os.path.join(session_dir, "binary")

    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(result_dir, exist_ok=True)
    os.makedirs(binary_dir, exist_ok=True)

    csv_path = os.path.join(session_dir, "capture_log.csv")

    print("Experiment session:", session_dir)
    print("Capture limit:", CAPTURE_LIMIT)
    print("Each received trigger frame will be logged immediately.")

    # Current camera full resolution 2592 x 1944, BGR = 3 bytes/pixel.
    buffer_size = 2592 * 1944 * 3
    data_buf = (c_ubyte * buffer_size)()

    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

    capture_index = 0
    centers = []

    with open(
            csv_path,
            "w",
            newline="",
            encoding="utf-8-sig"
    ) as csv_file:

        writer = csv.writer(csv_file)

        writer.writerow([
            "capture",
            "pc_time",
            "sdk_frame",
            "found",
            "center_u",
            "center_v",
            "bbox_x",
            "bbox_y",
            "bbox_w",
            "bbox_h",
            "area",
            "circularity",
            "otsu",
            "white_ratio",
            "raw_file",
            "result_file",
            "binary_file",
        ])
        csv_file.flush()

        while not g_bExit:

            memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

            ret = cam.MV_CC_GetImageForBGR(
                data_buf,
                buffer_size,
                stFrameInfo,
                1000
            )

            if ret != 0:
                # Timeout while waiting for hardware trigger is normal.
                continue

            capture_index += 1

            width = stFrameInfo.nWidth
            height = stFrameInfo.nHeight
            sdk_frame = int(stFrameInfo.nFrameNum)
            pc_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            print(
                "\n[%02d/%02d] get one frame: Width[%d], Height[%d], nFrameNum[%d]"
                % (
                    capture_index,
                    CAPTURE_LIMIT,
                    width,
                    height,
                    sdk_frame
                )
            )

            frame = np.ctypeslib.as_array(data_buf)
            frame = frame[:width * height * 3]
            frame = frame.reshape((height, width, 3)).copy()

            result_frame, info, binary = detect_black_dot(frame)

            tag = "capture_%02d_sdk%06d" % (
                capture_index,
                sdk_frame
            )

            raw_name = tag + ".png"
            result_name = tag + ".png"
            binary_name = tag + ".png"

            raw_path = os.path.join(raw_dir, raw_name)
            result_path = os.path.join(result_dir, result_name)
            binary_path = os.path.join(binary_dir, binary_name)

            save_png(raw_path, frame, "raw")
            save_png(result_path, result_frame, "result")
            save_png(binary_path, binary, "binary")

            if info["found"]:
                centers.append((info["u"], info["v"]))

            def csv_value(value, fmt=None):
                if value is None:
                    return ""
                if fmt is None:
                    return value
                return fmt % value

            writer.writerow([
                capture_index,
                pc_time,
                sdk_frame,
                info["found"],
                csv_value(info["u"], "%.4f"),
                csv_value(info["v"], "%.4f"),
                csv_value(info["bbox_x"]),
                csv_value(info["bbox_y"]),
                csv_value(info["bbox_w"]),
                csv_value(info["bbox_h"]),
                csv_value(info["area"], "%.2f"),
                csv_value(info["circularity"], "%.6f"),
                "%.2f" % info["otsu"],
                "%.6f" % info["white_ratio"],
                os.path.join("raw", raw_name),
                os.path.join("result", result_name),
                os.path.join("binary", binary_name),
            ])

            # Critical for the experiment: write every capture immediately.
            csv_file.flush()

            print("log updated:", csv_path)

            if info["found"]:
                print(
                    "logged center: u=%.4f, v=%.4f"
                    % (info["u"], info["v"])
                )
            else:
                print("logged as found=0; this capture is NOT deleted.")

            if capture_index >= CAPTURE_LIMIT:
                print(
                    "\n%d captures completed. Auto-stopping experiment."
                    % CAPTURE_LIMIT
                )
                g_bExit = True
                break

    write_repeatability_summary(
        session_dir,
        centers,
        capture_index
    )


if __name__ == "__main__":

    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("enum devices fail! ret[0x%x]" % ret)
        sys.exit()

    if deviceList.nDeviceNum == 0:
        print("find no device!")
        sys.exit()

    print("Find %d devices!" % deviceList.nDeviceNum)

    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nu3v device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("user serial number: %s" % strSerialNumber)

    nConnectionNum = input("please input the number of the device to connect:")

    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()

    # ch:创建相机实例 | en:Creat Camera Object
    cam = MvCamera()

    # ch:选择设备并创建句柄 | en:Select device and create handle
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
        nPacketSize = cam.MV_CC_GetOptimalPacketSize()
        if int(nPacketSize) > 0:
            ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
            if ret != 0:
                print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
        else:
            print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

    stBool = c_bool(False)
    ret = cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
    if ret != 0:
        print("get AcquisitionFrameRateEnable fail! ret[0x%x]" % ret)

    # ---------------------------------------------------------
    # Hardware trigger configuration
    # Same configuration verified successfully in MVS:
    #   Acquisition Mode = Continuous
    #   Trigger Selector = Frame Burst Start
    #   Burst Count      = 1
    #   Trigger Mode     = On
    #   Trigger Source   = Line0
    #   Trigger Edge     = Rising Edge
    # ---------------------------------------------------------

    ret = cam.MV_CC_SetEnumValueByString(
        "AcquisitionMode",
        "Continuous"
    )
    if ret != 0:
        print("set AcquisitionMode fail! ret[0x%x]" % ret)
        sys.exit()

    ret = cam.MV_CC_SetEnumValueByString(
        "TriggerSelector",
        "FrameBurstStart"
    )
    if ret != 0:
        print("set TriggerSelector fail! ret[0x%x]" % ret)
        sys.exit()

    ret = cam.MV_CC_SetIntValue(
        "AcquisitionBurstFrameCount",
        1
    )
    if ret != 0:
        print("set AcquisitionBurstFrameCount fail! ret[0x%x]" % ret)
        sys.exit()

    ret = cam.MV_CC_SetEnumValueByString(
        "TriggerMode",
        "On"
    )
    if ret != 0:
        print("set TriggerMode fail! ret[0x%x]" % ret)
        sys.exit()

    ret = cam.MV_CC_SetEnumValueByString(
        "TriggerSource",
        "Line0"
    )
    if ret != 0:
        print("set TriggerSource fail! ret[0x%x]" % ret)
        sys.exit()

    ret = cam.MV_CC_SetEnumValueByString(
        "TriggerActivation",
        "RisingEdge"
    )
    if ret != 0:
        print("set TriggerActivation fail! ret[0x%x]" % ret)
        sys.exit()

    print("Hardware trigger configured.")
    print("Waiting for Line0 rising edge...")

    # ch:开始取流 | en:Start grab image
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("start grabbing fail! ret[0x%x]" % ret)
        sys.exit()

    try:
        hThreadHandle = threading.Thread(target=work_thread, args=(cam, None, None))
        hThreadHandle.start()
    except:
        print("error: unable to start thread")

    print(
        "Press any key to stop early; "
        "otherwise the experiment auto-stops after %d captures."
        % CAPTURE_LIMIT
    )

    while not g_bExit:
        if msvcrt.kbhit():
            msvcrt.getch()
            g_bExit = True
            break

        time.sleep(0.05)

    hThreadHandle.join()

    # ch:停止取流 | en:Stop grab image
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("stop grabbing fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:关闭设备 | Close device
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("close deivce fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:销毁句柄 | Destroy handle
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("destroy handle fail! ret[0x%x]" % ret)
        sys.exit()
