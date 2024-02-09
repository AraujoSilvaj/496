import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

distance = [5, 7.5, 10, 12.5, 15, 17.5, 20, 25]
label_height = [0.26968, 0.076669, 0.059404, 0.06270, 0.05573, 0.05794, 0.06218, 0.064353]


'''
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
'''
# fitting nonlinear curve to find distance mapping
'''
# using only start and end points
d = [5, 25]
h = [0.26969, 0.064353]

# objective function to minimize
def objective_function(C):
	predicted_d = -.6 * C / np.tan(label_height / 2)
	return np.sum((distance - predicted_d)**2)

# initial guess for C
initial_guess = 1.0


# want to minimize the objective function
result = minimize(objective_function, initial_guess)

optimal_C = result.x[0]

print(optimal_C)
'''
# Generate x values for the fitted curve
fit_h = np.linspace(min(label_height), max(label_height), 100)


predicted_distance = 0.6 / np.tan(fit_h * 1.02 / 2)

plt.scatter(label_height, distance, label = 'Data')
plt.plot(fit_h, predicted_distance, 'r-', label = 'Predicted Values')

# Set labels and title
plt.xlabel('label_height')
plt.ylabel('Distance')
plt.title('Curve Fitting')
plt.legend()
plt.grid()

# Show the plot
plt.show()
