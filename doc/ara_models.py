import numpy as np
from matplotlib import pyplot as plt

teta = np.deg2rad(np.linspace(0,88,1000))

model_cos_aoi_dB = 10*np.log10(np.cos(teta))
model_sandy_gravel_dB = model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  1.5*np.exp(-(teta**2)/(np.deg2rad(30)**2)) -15
model_sandy_gravel_some_mud_dB = model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  4.5*np.exp(-(teta**2)/(np.deg2rad(25)**2)) -16
model_gravelly_muddy_sand_dB = model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  8.5*np.exp(-(teta**2)/(np.deg2rad(20)**2)) -18
model_muddy_sand_dB = model_cos_aoi_dB + 30*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**6))) +  12.5*np.exp(-(teta**2)/(np.deg2rad(13)**2)) -22
model_gravelly_mud_dB = model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  16.*np.exp(-(teta**2)/(np.deg2rad(5)**2)) -23
model_clay_dB =         model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  15.5*np.exp(-(teta**2)/(np.deg2rad(5)**2)) -26

model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  1.5*np.exp(-(teta**2)/(np.deg2rad(30)**2)) -15
model_cos_aoi_dB + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  8.5*np.exp(-(teta**2)/(np.deg2rad(20)**2)) -18

plt.plot(np.rad2deg(teta), model_clay_dB, label='clay')
plt.plot(np.rad2deg(teta), model_muddy_sand_dB, label='muddy sand')
plt.plot(np.rad2deg(teta), model_sandy_gravel_dB, label='sandy gravel')
plt.plot(np.rad2deg(teta), model_sandy_gravel_some_mud_dB, label='sandy gravel some_mud')
plt.plot(np.rad2deg(teta), model_gravelly_mud_dB, label='gravelly mud')
plt.plot(np.rad2deg(teta), model_gravelly_muddy_sand_dB, label='gravelly muddy sand')

plt.plot(np.rad2deg(teta), model_cos_aoi_dB-10, label='cos_aoi')
plt.xlabel('deg')
plt.ylabel('dB')
plt.grid(True)
plt.ylim([-45,-5])
plt.legend()
plt.show()
