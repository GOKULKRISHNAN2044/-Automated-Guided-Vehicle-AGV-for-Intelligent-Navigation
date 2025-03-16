import cv2
import numpy as np
import utlis

curveList = []
avgVal = 10


def detectDottedLines(edges, min_segment_length=20, max_gap_length=20, min_dotted_segments=3):
    """
    Detect dotted lines by checking for regular gaps between line segments.
    Args:
    edges: The edge-detected image.
    min_segment_length: The minimum length of a segment to be considered as part of a dotted line.
    max_gap_length: The maximum length of a gap to be considered as part of a dotted line.
    min_dotted_segments: The minimum number of dotted segments required to consider the line as dotted.

    Returns:
    bool: True if dotted lines are detected, False otherwise.
    """
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=min_segment_length,
                            maxLineGap=max_gap_length)
    if lines is None:
        return False

    # Sort lines by their position along the x-axis (assuming mostly vertical lines)
    lines = sorted(lines, key=lambda line: line[0][0])

    segment_lengths = []
    gaps = []

    # Calculate lengths of segments and gaps
    for i in range(len(lines) - 1):
        x1, y1, x2, y2 = lines[i][0]
        x1_next, y1_next, x2_next, y2_next = lines[i + 1][0]

        length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        gap = np.sqrt((x1_next - x2) ** 2 + (y1_next - y2) ** 2)

        segment_lengths.append(length)
        gaps.append(gap)

    # Check for regular gaps indicating a dotted line
    dotted_segments = 0
    for gap in gaps:
        if gap <= max_gap_length:
            dotted_segments += 1
        else:
            dotted_segments = 0  # Reset if gap is too large

        if dotted_segments >= min_dotted_segments:
            return True

    return False


def getLaneCurve(img, display=2):
    imgCopy = img.copy()
    imgResult = img.copy()
    imgCopy = cv2.resize(imgCopy, (480, 240))

    # Step 1: Thresholding
    imgThres = utlis.thresholding(img)

    # Step 2: Perspective Warp
    hT, wT, c = img.shape
    points = utlis.valTrackbars()
    imgWarp = utlis.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utlis.drawPoints(imgCopy, points)

    # Step 3: Histogram for Lane Detection
    midPoint, imgHist = utlis.getHistogram(imgWarp, True, 0.5, region=4)
    curveAveragePoint, imgHist = utlis.getHistogram(imgWarp, True, 0.9)
    curveRaw = curveAveragePoint - midPoint

    # Step 4: Smooth the Curve
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    # Step 5: Edge Detection
    imgEdges = utlis.getEdges(img)
    imgWarpEdges = utlis.warpImg(imgEdges, points, wT, hT)  # Apply perspective warp to edges

    # Detect Dotted Lines
    if detectDottedLines(imgWarpEdges):
        print("Dotted lines found")

    # Step 6: Create Edge Overlay on Original Image
    imgOverlay = img.copy()
    imgOverlay[imgEdges > 0] = (0, 255, 0)  # Highlight edges in green
    imgWarpOverlay = utlis.warpImg(imgOverlay, points, wT, hT)  # Apply perspective warp to overlay

    # Step 7: Display Results
    if display != 0:
        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10), (w * x + int(curve // 50), midY + 10),
                     (0, 0, 255), 2)

    if display == 2:
        imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp, imgWarpEdges],  # Display warped edges
                                             [imgHist, imgLaneColor, imgResult, imgWarpOverlay]))
        cv2.imshow('ImageStack', imgStacked)
    elif display == 1:
        cv2.imshow('Result', imgResult)

    curve = curve / 100
    if curve > 1: curve = 1
    if curve < -1: curve = -1

    return curve


if __name__ == '__main__':
    cap = cv2.VideoCapture('vid1.mp4')
    frameCounter = 0
    intialTracbarVals = [102, 80, 20, 214]
    utlis.initializeTrackbars(intialTracbarVals)
    while True:
        frameCounter += 1
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0
        _, img = cap.read()
        img = cv2.resize(img, (480, 240))
        curve = getLaneCurve(img, display=2)
        print(curve)
        cv2.imshow("sample", img)
        cv2.waitKey(1)