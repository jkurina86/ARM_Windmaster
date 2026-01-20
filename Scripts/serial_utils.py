#!/usr/bin/env python3
"""
Cross-platform serial port utilities for Windows and Linux.
"""

import sys
from serial.tools import list_ports


def list_serial_ports():
    """
    List all available serial ports on the system.
    Returns list of tuples: (device, description)
    """
    ports = []
    for port in list_ports.comports():
        ports.append((port.device, port.description))
    return sorted(ports, key=lambda x: x[0])


def select_port_menu(default_baud: int = 115200) -> str:
    """
    Display interactive menu for serial port selection.
    Returns selected port device string, or exits if user cancels.
    """
    ports = list_serial_ports()

    if not ports:
        print("No serial ports found!")
        print("Make sure the device is connected and drivers are installed.")
        sys.exit(1)

    print("\nAvailable serial ports:")
    print("-" * 60)
    for i, (device, description) in enumerate(ports, 1):
        print(f"  {i}. {device} - {description}")
    print("-" * 60)

    while True:
        try:
            choice = input(f"\nSelect port (1-{len(ports)}) or 'q' to quit: ").strip()

            if choice.lower() == 'q':
                print("Cancelled.")
                sys.exit(0)

            index = int(choice) - 1
            if 0 <= index < len(ports):
                selected = ports[index][0]
                print(f"Selected: {selected}")
                return selected
            else:
                print(f"Please enter a number between 1 and {len(ports)}")
        except ValueError:
            print("Invalid input. Enter a number or 'q' to quit.")
        except KeyboardInterrupt:
            print("\nCancelled.")
            sys.exit(0)


def get_port(port_arg: str = None) -> str:
    """
    Get serial port - either from command line argument or interactive menu.

    Args:
        port_arg: Optional port from command line (e.g., 'COM6' or '/dev/ttyUSB0')

    Returns:
        Selected port device string
    """
    if port_arg:
        # Validate the provided port exists
        available = [p[0] for p in list_serial_ports()]
        if port_arg in available:
            return port_arg
        else:
            print(f"Warning: Port '{port_arg}' not found in available ports.")
            print("Available ports:", ', '.join(available) if available else "None")
            response = input("Use anyway? (y/N): ").strip().lower()
            if response == 'y':
                return port_arg
            # Fall through to menu selection

    return select_port_menu()
