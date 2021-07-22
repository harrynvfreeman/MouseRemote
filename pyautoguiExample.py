import pyautogui
import time

pyautogui.PAUSE = 0.001

start = time.time()
pyautogui.moveTo(860, 540, duration=0)
end = time.time()

print(end-start)

start = time.time()
pyautogui.moveTo(20, 20, duration=0)
end = time.time()

print(end-start)