import matplotlib.pyplot as plt

def manchester_encode(input_string):
    encoded_signal = []

    for char in input_string:
        binary_char = bin(ord(char))[2:].zfill(8)
        for bit in binary_char:
            if bit == '0':
                encoded_signal.extend([0, 1])
            else:
                encoded_signal.extend([1, 0])

    return encoded_signal

def plot_manchester_signal(encoded_signal):
    plt.plot(encoded_signal, drawstyle='steps-pre')
    plt.title('Manchester Encoded Signal')
    plt.xlabel('Time')
    plt.ylabel('Voltage Level')
    plt.yticks([0, 1], ['Low', 'High'])
    plt.show()

if __name__ == "__main__":
    # Input ASCII string
    input_string = input("Enter ASCII string: ")

    # Encode the string using Manchester encoding
    encoded_signal = manchester_encode(input_string)

    # Plot the Manchester-encoded signal
    plot_manchester_signal(encoded_signal)
