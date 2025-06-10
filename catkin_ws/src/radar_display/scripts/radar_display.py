#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import PointStamped
import signal

tracks_data = {}

def track_callback(msg, track_name):
    # Update track info
    tracks_data[track_name] = (msg.point.x, msg.point.y, msg.point.z)

def draw_box(stdscr, top, left, height, width, title=""):
    if height < 3 or width < 4:
        return  # too small to draw box

    # Draw top border
    stdscr.addstr(top, left, "+" + "-"*(width-2) + "+")
    # Draw bottom border
    stdscr.addstr(top + height - 1, left, "+" + "-"*(width-2) + "+")
    # Draw sides
    for y in range(top+1, top + height -1):
        stdscr.addstr(y, left, "|")
        stdscr.addstr(y, left + width -1, "|")

    # Draw title if any
    if title:
        title_str = f"[ {title} ]"
        max_title_len = width - 4
        if len(title_str) > max_title_len:
            title_str = title_str[:max_title_len]
        stdscr.addstr(top, left + 2, title_str)

def curses_main(stdscr):
    curses.curs_set(0)
    
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)  # Green text, default background
    GREEN_PAIR = curses.color_pair(1)

    
    stdscr.nodelay(True)
    stdscr.timeout(200)
    
    height, width = stdscr.getmaxyx()

    while not rospy.is_shutdown():
        new_height, new_width = stdscr.getmaxyx()
        if curses.is_term_resized(height, width):
            height, width = new_height, new_width
            curses.resize_term(height, width)
            stdscr.erase()

        stdscr.erase()
        height, width = stdscr.getmaxyx()

        mid_x = width // 2
        stdscr.addstr(0, 2, "L: Left Radar Tracks")
        stdscr.addstr(0, mid_x + 2, "R: Right Radar Tracks")

        left_x = 2
        right_x = mid_x + 2

        left_groups = []
        right_groups = []

        for track_num in range(1, 11):
            left_lines = []
            right_lines = []
            for subtrack in range(1, 7):
                # Left
                track_name = f"track_L{track_num}_{subtrack}"
                x, y, z = tracks_data.get(track_name, (0, 0, 0))
                left_lines.append((track_name, f"{track_name}: LONG={x:.2f}, LAT={y:.2f}, FLAG={z:.0f}", int(z) == 1))

                # Right
                track_name = f"track_R{track_num}_{subtrack}"
                x, y, z = tracks_data.get(track_name, (0, 0, 0))
                right_lines.append((track_name, f"{track_name}: LONG={x:.2f}, LAT={y:.2f}, FLAG={z:.0f}", int(z) == 1))

            left_groups.append((f"L{track_num}", left_lines))
            right_groups.append((f"R{track_num}", right_lines))

        box_height = 8
        spacing = 1
        total_box_height = box_height + spacing

        max_groups = max(0, (height - 1) // total_box_height)
        max_left_width = mid_x - 4
        max_right_width = mid_x - 4

        current_y = 1
        for group_name, lines in left_groups[:max_groups]:
            if current_y + box_height > height:
                break
            min_box_w = max(len(f"[ {group_name} ]") + 4, 20)  # Ensure room for title and box
            box_w = max(min_box_w, min(max(len(line_text) for _, line_text, _ in lines) + 4, max_left_width))
            draw_box(stdscr, current_y, left_x, box_height, box_w, title=group_name)
            for i, (track_name, line_text, is_valid) in enumerate(lines):
                attr = GREEN_PAIR if is_valid else curses.A_NORMAL
                stdscr.addstr(current_y + 1 + i, left_x + 2, line_text[:box_w - 4], attr)
            current_y += total_box_height

        current_y = 1
        for group_name, lines in right_groups[:max_groups]:
            if current_y + box_height > height:
                break
            min_box_w = max(len(f"[ {group_name} ]") + 4, 20)  # Ensure room for title and box
            box_w = max(min_box_w, min(max(len(line_text) for _, line_text, _ in lines) + 4, max_right_width))
            draw_box(stdscr, current_y, right_x, box_height, box_w, title=group_name)
            for i, (track_name, line_text, is_valid) in enumerate(lines):
                attr = GREEN_PAIR if is_valid else curses.A_NORMAL
                stdscr.addstr(current_y + 1 + i, right_x + 2, line_text[:box_w - 4], attr)
            current_y += total_box_height

        stdscr.refresh()

        try:
            key = stdscr.getch()
            if key == ord('q'):
                break
        except curses.error:
            pass

        
def main():
    rospy.init_node('radar_display_node')

    track_topics = [
        'car/radar/track_L1_1',
        'car/radar/track_L1_2',
        'car/radar/track_L1_3',
        'car/radar/track_L1_4',
        'car/radar/track_L1_5',
        'car/radar/track_L1_6',
        'car/radar/track_R1_1',
        'car/radar/track_R1_2',
        'car/radar/track_R1_3',
        'car/radar/track_R1_4',
        'car/radar/track_R1_5',
        'car/radar/track_R1_6',
        
        'car/radar/track_L2_1',
        'car/radar/track_L2_2',
        'car/radar/track_L2_3',
        'car/radar/track_L2_4',
        'car/radar/track_L2_5',
        'car/radar/track_L2_6',
        'car/radar/track_R2_1',
        'car/radar/track_R2_2',
        'car/radar/track_R2_3',
        'car/radar/track_R2_4',
        'car/radar/track_R2_5',
        'car/radar/track_R2_6',
        
        'car/radar/track_L3_1',
        'car/radar/track_L3_2',
        'car/radar/track_L3_3',
        'car/radar/track_L3_4',
        'car/radar/track_L3_5',
        'car/radar/track_L3_6',
        'car/radar/track_R3_1',
        'car/radar/track_R3_2',
        'car/radar/track_R3_3',
        'car/radar/track_R3_4',
        'car/radar/track_R3_5',
        'car/radar/track_R3_6',
        
        'car/radar/track_L4_1',
        'car/radar/track_L4_2',
        'car/radar/track_L4_3',
        'car/radar/track_L4_4',
        'car/radar/track_L4_5',
        'car/radar/track_L4_6',
        'car/radar/track_R4_1',
        'car/radar/track_R4_2',
        'car/radar/track_R4_3',
        'car/radar/track_R4_4',
        'car/radar/track_R4_5',
        'car/radar/track_R4_6',
        
        'car/radar/track_L5_1',
        'car/radar/track_L5_2',
        'car/radar/track_L5_3',
        'car/radar/track_L5_4',
        'car/radar/track_L5_5',
        'car/radar/track_L5_6',
        'car/radar/track_R5_1',
        'car/radar/track_R5_2',
        'car/radar/track_R5_3',
        'car/radar/track_R5_4',
        'car/radar/track_R5_5',
        'car/radar/track_R5_6',
        
        'car/radar/track_L6_1',
        'car/radar/track_L6_2',
        'car/radar/track_L6_3',
        'car/radar/track_L6_4',
        'car/radar/track_L6_5',
        'car/radar/track_L6_6',
        'car/radar/track_R6_1',
        'car/radar/track_R6_2',
        'car/radar/track_R6_3',
        'car/radar/track_R6_4',
        'car/radar/track_R6_5',
        'car/radar/track_R6_6',
        
        'car/radar/track_L7_1',
        'car/radar/track_L7_2',
        'car/radar/track_L7_3',
        'car/radar/track_L7_4',
        'car/radar/track_L7_5',
        'car/radar/track_L7_6',
        'car/radar/track_R7_1',
        'car/radar/track_R7_2',
        'car/radar/track_R7_3',
        'car/radar/track_R7_4',
        'car/radar/track_R7_5',
        'car/radar/track_R7_6',
        
        'car/radar/track_L8_1',
        'car/radar/track_L8_2',
        'car/radar/track_L8_3',
        'car/radar/track_L8_4',
        'car/radar/track_L8_5',
        'car/radar/track_L8_6',
        'car/radar/track_R8_1',
        'car/radar/track_R8_2',
        'car/radar/track_R8_3',
        'car/radar/track_R8_4',
        'car/radar/track_R8_5',
        'car/radar/track_R8_6',
        
        'car/radar/track_L9_1',
        'car/radar/track_L9_2',
        'car/radar/track_L9_3',
        'car/radar/track_L9_4',
        'car/radar/track_L9_5',
        'car/radar/track_L9_6',
        'car/radar/track_R9_1',
        'car/radar/track_R9_2',
        'car/radar/track_R9_3',
        'car/radar/track_R9_4',
        'car/radar/track_R9_5',
        'car/radar/track_R9_6',
        
        'car/radar/track_L10_1',
        'car/radar/track_L10_2',
        'car/radar/track_L10_3',
        'car/radar/track_L10_4',
        'car/radar/track_L10_5',
        'car/radar/track_L10_6',
        'car/radar/track_R10_1',
        'car/radar/track_R10_2',
        'car/radar/track_R10_3',
        'car/radar/track_R10_4',
        'car/radar/track_R10_5',
        'car/radar/track_R10_6',
    ]

    for topic in track_topics:
        rospy.Subscriber(topic, PointStamped, track_callback, callback_args=topic.split('/')[-1])

    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()
