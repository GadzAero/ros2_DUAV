import curses
import threading
import time

def action_process(screen):
    while True:
        screen.addstr(1, 0, "Action process is running...")
        screen.refresh()
        time.sleep(2)  # Simulate some work with sleep

def menu_process(screen):
    screen.addstr(0, 0, "Enter a command: ")
    screen.refresh()
    while True:
        user_input = screen.getstr(0, 16).decode('utf-8')
        if user_input == "stop":
            screen.addstr(2, 0, "Stopping the action process...")
            screen.refresh()
            break

def main(screen):
    # Create threads for each process
    action_thread = threading.Thread(target=action_process, args=(screen,))
    menu_thread = threading.Thread(target=menu_process, args=(screen,))

    # Start the threads
    action_thread.start()
    menu_thread.start()

    # Wait for the menu thread to finish
    # menu_thread.join()

curses.wrapper(main)