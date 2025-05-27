from hub75 import Hub75

import os

font_saver=Hub75()
# 
# bdf_font_file="/fonts/Frontier-10.bdf"
mini_font_file="/fonts/Frontier-10.mfont"
# 
# print("Loading",bdf_font_file)
# font_saver.font = font_saver.load_bdf_font(bdf_font_file)
# print("Saving",mini_font_file)
# font_saver.save_minifont(mini_font_file)
# print("Completed")

print("Test font")
font_saver.font = font_saver.load_minifont(mini_font_file)
font_saver.font=font_saver.monospace_digits(font_saver.font)

font_saver.hline(0,15,64,0,255,0)
font_saver.draw_text(0,15, "01234567890")
font_saver.refresh()

font_saver.font=font_saver.monospace_digits(font_saver.font)

font_saver.hline(0,15,64,0,255,0)
font_saver.draw_text(0,15, "01234567890")
font_saver.refresh()
