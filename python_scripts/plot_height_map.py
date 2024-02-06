import matplotlib.pyplot as plt
import numpy as np

distance = [7.5, 10, 12.5, 15, 17.5, 20, 25]
label_height = [0.076669, 0.059404, 0.06270, 0.05573, 0.05794, 0.06218, 0.064353]

# Fit a polynomial of degree 2 (you can adjust the degree as needed)
coefficients = np.polyfit(distance, label_height, 2)
poly = np.poly1d(coefficients)

# Generate x values for the fitted curve
fit_x = np.linspace(min(distance), max(distance), 100)

# Evaluate the polynomial at the generated x values
fit_y = poly(fit_x)

plt.plot(distance,label_height, 'b*', label='Data')

# Plot the fitted curve
plt.plot(fit_x, fit_y, 'r-', label='Polyfit Curve')
plt.xlabel('Distance [ft]')
plt.ylabel('Pixel Height []')
plt.title('Label height to distance mapping (indoors)')
plt.legend()
plt.show()
