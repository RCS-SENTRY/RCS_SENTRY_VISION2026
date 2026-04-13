# =============================================================================
# 单个装甲板检测结果
# =============================================================================
std_msgs/Header header

uint8 RED=0
uint8 BLUE=1
uint8 NONE=2

uint8 color              # 装甲板颜色: 0=RED, 1=BLUE, 2=NONE
uint8 class_id           # YOLO 类别 ID（映射到具体编号/类型）
string label             # 人类可读标签，如 "blue_sentry_small"
float32 confidence       # 检测置信度 [0, 1]

# 四个灯条角点像素坐标（顺序：左上、右上、右下、左下）
geometry_msgs/Point[4] apexes

# 边界框（像素坐标）
uint32 bbox_x
uint32 bbox_y
uint32 bbox_w
uint32 bbox_h