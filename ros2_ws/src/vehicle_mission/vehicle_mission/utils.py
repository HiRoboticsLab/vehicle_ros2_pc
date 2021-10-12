import cv2
import requests


# 使用前，电脑端需要执行 “pip3 install requests”
def detect_traffic_light(image):
    # 显示图片
    # cv2.imshow('camera', image)
    try:
        if not (image is None):
            # result_img = np.copy(image)
            response = requests.post('http://127.0.0.1:24401/', params={'threshold': 0.1},
                                     data=bytes(cv2.imencode('.jpg', image)[1])).json()
            print(response)
            if len(response['results']) != 0:
                # print(response['results'])
                max_confidence, max_index = 0, 0
                for index, result in enumerate(response['results']):
                    # print(result)
                    confidence = result['confidence']
                    if confidence > max_confidence:
                        max_confidence = confidence
                        max_index = index
                result = response['results'][max_index]
                confidence = result['confidence']
                label = result['label']
                x = result['location']['left']
                y = result['location']['top']
                w = result['location']['width']
                h = result['location']['height']
                # 输入参数为图像、左上角坐标、右下角坐标、颜色(B,G,R)数组、粗细
                image = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
                # 输入参数为图像、文本、位置、字体、大小、颜色(B,G,R)数组、粗细
                image = cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # print(label, confidence, x, y, w, h)
                # cv2.imshow('traffic_light', image)
                return label, x, y, w, h, image 
    except Exception as e:
        print(e)

