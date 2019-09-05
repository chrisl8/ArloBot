#!/usr/bin/env python
# -*- coding: latin-1 -*-
import curses
import curses.textpad
from time import sleep


class Screen(object):
    UP = -1
    DOWN = 1

    def __init__(self, items, sendCommandsToSerialTester):
        """ Initialize the screen window

        Attributes
            window: A full curses screen window

            width: The width of `window`
            height: The height of `window`

            max_lines: Maximum visible line count for `result_window`
            top: Available top line position for current page (used on scrolling)
            bottom: Available bottom line position for whole pages (as length of items)
            current: Current highlighted line number (as window cursor)
            page: Total page count which being changed corresponding to result of a query (starts from 0)

            ┌--------------------------------------┐
            |1. Item                               |
            |--------------------------------------| <- top = 1
            |2. Item                               |
            |3. Item                               |
            |4./Item///////////////////////////////| <- current = 3
            |5. Item                               |
            |6. Item                               |
            |7. Item                               |
            |8. Item                               | <- max_lines = 7
            |--------------------------------------|
            |9. Item                               |
            |10. Item                              | <- bottom = 10
            |                                      |
            |                                      | <- page = 1 (0 and 1)
            └--------------------------------------┘

        Returns
            None
        """
        self.width = 0
        self.height = 0
        self.headerHeight = 1
        self.borderWidth = 2
        self.top = 0
        self.current = 0
        self.current_page = 1
        self.maximumLines = 1000
        self.items = items
        self.maxLineNumber = len(self.items)
        self.sendCommandsToSerialTester = sendCommandsToSerialTester
        self._statusLine = ""
        self._odomLineOne = ""
        self._odomLineTwo = ""
        self._odomLineThree = ""
        self._odomLineFour = ""
        self._odomLineFive = ""
        self._statusErrorCount = 0
        self._serialWriteDelay = "?"
        self._goodPacketDelay = "?"
        self._testPacketsSent = "?"
        self._testPacketsReceived = "?"
        self._sendingTwistCommands = False
        self._sendingLedCommands = False
        self._updatingSettings = False
        self._performingTestManeuvers = False
        self._overridingPosition = False
        self._enteringPositionOverride = None
        self._acceptablePositionCharacters = [
            ord("."),
            ord("1"),
            ord("2"),
            ord("3"),
            ord("4"),
            ord("5"),
            ord("6"),
            ord("7"),
            ord("8"),
            ord("9"),
            ord("0"),
        ]
        self._positionOverrideData = ""
        self._scrollingOutputWindow = None
        self._stayAtEnd = True
        self._max_lines = None
        self.window = None
        self.maximumScrollPage = None

    def init_curses(self):
        """Setup the curses"""
        curses.curs_set(0)
        self.window.nodelay(1)  # set getch() non-blocking

        def maxLineDisplayCount():
            maxLineCount = self._scrollingOutputWindow.getmaxyx()
            maxLineCount = maxLineCount[0] - self.borderWidth
            return maxLineCount

        self._max_lines = maxLineDisplayCount

        curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_CYAN)

        self.height, self.width = self.window.getmaxyx()

        self._scrollingOutputWindow = self.window.subwin(self.headerHeight, 0)

    def run(self, window):
        """Continue running the TUI until get interrupted"""
        self.window = window
        self.init_curses()
        self.maximumScrollPage = len(self.items) // self._max_lines()
        try:
            self.input_stream()
        except KeyboardInterrupt:
            pass
        finally:
            curses.endwin()

    def input_stream(self):
        """Waiting an input and run a proper method according to type of input"""
        while True:
            self.display()
            sleep(0.05)  # Because getch() is non-blocking now (due to nodelay(1))

            ch = self.window.getch()
            curses.flushinp()
            # curses does not have an __all__ statement.
            # noinspection PyUnresolvedReferences
            if ch == curses.KEY_RESIZE:
                self.height, self.width = self.window.getmaxyx()
                if self.height > self.headerHeight:
                    self._scrollingOutputWindow.resize(
                        self.height - self.headerHeight, self.width
                    )
            elif ch == curses.KEY_UP:
                self.scroll(self.UP)
                self._stayAtEnd = False
            elif ch == curses.KEY_DOWN:
                self.scroll(self.DOWN)
                self._stayAtEnd = False
            elif ch == curses.KEY_PPAGE:
                self.paging(self.UP)
                self._stayAtEnd = False
            elif ch == curses.KEY_NPAGE:
                self.paging(self.DOWN)
                self._stayAtEnd = False
            elif ch == curses.KEY_HOME:
                self.current = 0
                self.top = 0
                self.current_page = 1
                self._stayAtEnd = False
            elif ch == curses.KEY_END:
                if len(self.items) > self._max_lines():
                    self.top = len(self.items) - self._max_lines()
                    self.current = self._max_lines() - 1
                else:
                    self.current = len(self.items) - 1
                self._stayAtEnd = True
            elif self._updatingSettings:
                if ch == ord("w"):
                    self.sendCommandsToSerialTester("settings_trackWidthDown")
                elif ch == ord("W"):
                    self.sendCommandsToSerialTester("settings_trackWidthUp")
                elif ch == ord("d"):
                    self.sendCommandsToSerialTester("settings_distancePerCountDown")
                elif ch == ord("D"):
                    self.sendCommandsToSerialTester("settings_distancePerCountUp")
                elif ch == ord("o"):
                    self.sendCommandsToSerialTester("settings_abdSpeedLimitDown")
                elif ch == ord("O"):
                    self.sendCommandsToSerialTester("settings_abdSpeedLimitUp")
                elif ch == ord("l"):
                    self.sendCommandsToSerialTester("settings_abdRSpeedLimitDown")
                elif ch == ord("L"):
                    self.sendCommandsToSerialTester("settings_abdRSpeedLimitUp")
                elif ch == ord("a"):
                    self.sendCommandsToSerialTester("settings_ignoreProximity")
                elif ch == ord("c"):
                    self.sendCommandsToSerialTester("settings_ignoreCliffSensors")
                elif ch == ord("i"):
                    self.sendCommandsToSerialTester("settings_ignoreIRSensors")
                elif ch == ord("f"):
                    self.sendCommandsToSerialTester("settings_ignoreFloorSensors")
                elif ch == ord("p"):
                    self.sendCommandsToSerialTester("settings_pluggedIn")
                elif ch == ord("q"):
                    self._updatingSettings = False
            elif self._sendingTwistCommands:
                if ch == ord("u"):
                    self.sendCommandsToSerialTester("moveu")
                elif ch == ord("i"):
                    self.sendCommandsToSerialTester("movei")
                elif ch == ord("o"):
                    self.sendCommandsToSerialTester("moveo")
                elif ch == ord("j"):
                    self.sendCommandsToSerialTester("movej")
                elif ch == ord("k"):
                    self.sendCommandsToSerialTester("movek")
                elif ch == ord("l"):
                    self.sendCommandsToSerialTester("movel")
                elif ch == ord("m"):
                    self.sendCommandsToSerialTester("movem")
                elif ch == ord(","):
                    self.sendCommandsToSerialTester("move,")
                elif ch == ord("."):
                    self.sendCommandsToSerialTester("move.")
                elif ch == ord("a"):
                    self.sendCommandsToSerialTester("movea")
                elif ch == ord("z"):
                    self.sendCommandsToSerialTester("movez")
                elif ch == ord("s"):
                    self.sendCommandsToSerialTester("moves")
                elif ch == ord("x"):
                    self.sendCommandsToSerialTester("movex")
                elif ch == ord("d"):
                    self.sendCommandsToSerialTester("moved")
                elif ch == ord("c"):
                    self.sendCommandsToSerialTester("movec")
                elif ch == ord("q"):
                    self._sendingTwistCommands = False
            elif self._sendingLedCommands:
                if ch == ord("0"):
                    self.sendCommandsToSerialTester("led_0")
                elif ch == ord("1"):
                    self.sendCommandsToSerialTester("led_1")
                elif ch == ord("2"):
                    self.sendCommandsToSerialTester("led_2")
                elif ch == ord("3"):
                    self.sendCommandsToSerialTester("led_3")
                elif ch == ord("4"):
                    self.sendCommandsToSerialTester("led_4")
                elif ch == ord("q"):
                    self._sendingLedCommands = False
            elif self._performingTestManeuvers:
                if ch == ord("f"):
                    self.sendCommandsToSerialTester("maneuvers_forward")
                elif ch == ord("b"):
                    self.sendCommandsToSerialTester("maneuvers_backward")
                elif ch == ord("i"):
                    self.sendCommandsToSerialTester("maneuvers_cancel")
                elif ch == ord("4"):
                    self.sendCommandsToSerialTester("maneuvers_rotate_45")
                elif ch == ord("9"):
                    self.sendCommandsToSerialTester("maneuvers_rotate_90")
                elif ch == ord("8"):
                    self.sendCommandsToSerialTester("maneuvers_rotate_180")
                elif ch == ord("3"):
                    self.sendCommandsToSerialTester("maneuvers_rotate_360")
                elif ch == ord("r"):
                    self.sendCommandsToSerialTester("maneuvers_reverse")
                elif ch == ord("a"):
                    self.sendCommandsToSerialTester("movea")
                elif ch == ord("z"):
                    self.sendCommandsToSerialTester("movez")
                elif ch == ord("s"):
                    self.sendCommandsToSerialTester("moves")
                elif ch == ord("x"):
                    self.sendCommandsToSerialTester("movex")
                elif ch == ord("d"):
                    self.sendCommandsToSerialTester("moved")
                elif ch == ord("c"):
                    self.sendCommandsToSerialTester("movec")
                elif ch == ord("q"):
                    self._performingTestManeuvers = False
            elif self._overridingPosition:
                if self._enteringPositionOverride is not None:
                    if ch in self._acceptablePositionCharacters:
                        self._positionOverrideData = self._positionOverrideData + chr(
                            ch
                        )
                    elif ch == ord("q"):
                        self._enteringPositionOverride = None
                    elif ch == 10 or ch == curses.KEY_ENTER:
                        self.sendCommandsToSerialTester(
                            "overridePosition_"
                            + self._enteringPositionOverride
                            + "_"
                            + self._positionOverrideData
                        )
                        self._enteringPositionOverride = None
                elif ch == ord("x"):
                    self._positionOverrideData = ""
                    self._enteringPositionOverride = "X"
                elif ch == ord("y"):
                    self._positionOverrideData = ""
                    self._enteringPositionOverride = "Y"
                elif ch == ord("h"):
                    self._positionOverrideData = ""
                    self._enteringPositionOverride = "Heading"
                elif ch == ord("q"):
                    self._positionOverrideData = ""
                    self._overridingPosition = False
            elif ch == ord("t"):
                self.sendCommandsToSerialTester("genericTest")
            elif ch == ord("r"):
                self.sendCommandsToSerialTester("speedTestRawBps")
            elif ch == ord("i"):
                self.sendCommandsToSerialTester("interrupt")
            elif ch == ord("m"):
                self._sendingTwistCommands = True
            elif ch == ord("s"):
                self._updatingSettings = True
            elif ch == ord("a"):
                self._performingTestManeuvers = True
            elif ch == ord("p"):
                self._overridingPosition = True
            elif ch == ord("l"):
                self._sendingLedCommands = True
            elif ch == ord("q"):
                self.sendCommandsToSerialTester("quit")
                break
            elif ch == curses.ascii.ESC:
                self.sendCommandsToSerialTester("quit")
                break

    def addLine(self, line):
        if len(self.items) >= self.maximumLines:
            self.items = self.items[1:]  # Shift
        self.items.append(line)
        if self._max_lines is not None and self._max_lines() > 0:
            self.maximumScrollPage = len(self.items) // self._max_lines()

    def setStatusLine(self, line):
        self._statusLine = line

    def setOdomLines(self, line1, line2, line3, line4, line5):
        self._odomLineOne = line1
        self._odomLineTwo = line2
        self._odomLineThree = line3
        self._odomLineFour = line4
        self._odomLineFive = line5

    def inputStatusItems(
        self,
        errorCount=0,
        serialWriteDelay=0,
        goodPacketDelay=0,
        testPacketsSent=0,
        testPacketsReceived=0,
    ):
        self._statusErrorCount = errorCount
        self._serialWriteDelay = serialWriteDelay
        self._goodPacketDelay = goodPacketDelay
        self._testPacketsReceived = testPacketsReceived
        self._testPacketsSent = testPacketsSent

    def scroll(self, direction):
        """Scrolling the window when pressing up/down arrow keys"""
        # next cursor position after scrolling
        next_line = self.current + direction

        # Up direction scroll overflow
        # current cursor position is 0, but top position is greater than 0
        if (direction == self.UP) and (self.top > 0 and self.current == 0):
            self.top += direction
            return
        # Down direction scroll overflow
        # next cursor position touch the max lines, but absolute position of max lines could not touch the bottom
        if (
            (direction == self.DOWN)
            and (next_line == self._max_lines())
            and (self.top + self._max_lines() < len(self.items))
        ):
            self.top += direction
            return
        # Scroll up
        # current cursor position or top position is greater than 0
        if (direction == self.UP) and (self.top > 0 or self.current > 0):
            self.current = next_line
            return
        # Scroll down
        # next cursor position is above max lines, and absolute position of next cursor could not touch the bottom
        if (
            (direction == self.DOWN)
            and (next_line < self._max_lines())
            and (self.top + next_line < len(self.items))
        ):
            self.current = next_line
            return

    def paging(self, direction):
        """Paging the window when pressing left/right arrow keys"""
        current_page = (self.top + self.current) // self._max_lines()
        next_page = current_page + direction
        # The last page may have fewer items than max lines,
        # so we should adjust the current cursor position as maximum item count on last page
        if next_page == self.maximumScrollPage:
            self.current = min(self.current, len(self.items) % self._max_lines() - 1)
        if self.current < 0:
            self.current = 0

        # Page up
        # if current page is not a first page, page up is possible
        # top position can not be negative, so if top position is going to be negative, we should set it as 0
        if (direction == self.UP) and (current_page > 0):
            self.top = max(0, self.top - self._max_lines())
            self.current_page = next_page + 1
            return
        # If we are at the first page, but line 0 is not displayed, scroll up
        if (direction == self.UP) and (current_page == 0) and (self.top > 0):
            self.top = 0
            return
        # If we are at top, but the cursor is not in the top line, move to it
        if (direction == self.UP) and (current_page == 0) and (self.current > 0):
            self.current = 0
            return

        # Page down
        # if current page is not a last page, page down is possible
        if (direction == self.DOWN) and (current_page < self.maximumScrollPage):
            self.top += self._max_lines()
            self.current_page = next_page + 1
            return
        # If we are at the last page, but not all lines are in display, scroll down
        if (
            (direction == self.DOWN)
            and (current_page == self.maximumScrollPage)
            and (self.top < (len(self.items) - self._max_lines()))
        ):
            self.top = len(self.items) - self._max_lines()
            return
        # If we are at the last page, but not the last line, move to the last line
        if (
            (direction == self.DOWN)
            and (current_page == self.maximumScrollPage)
            and (self.current < len(self.items) - self.top - 1)
        ):
            self.current = len(self.items) - self.top - 1
            return

    def displayRow(self, rowNumber, line):
        if line != "" and self.height > rowNumber + 1:
            self.window.addstr(rowNumber, 0, line)
            if len(line) > self.width:
                rowNumber += 1
            rowNumber += 1
        return rowNumber

    def display(self):
        """Display the items on window"""
        self.window.erase()
        rowNumber = 0
        rowNumber = self.displayRow(
            rowNumber, "\tArloBot Propeller Serial Interface Test Program"
        )
        rowNumber = self.displayRow(rowNumber, self._statusLine)
        rowNumber = self.displayRow(
            rowNumber,
            "Errors: "
            + str(self._statusErrorCount)
            + " | Write Delay: "
            + str(self._serialWriteDelay)
            + " | Update Delay: "
            + str(self._goodPacketDelay)
            + " | Test Sent:"
            + str(self._testPacketsSent)
            + "/Rcvd: "
            + str(self._testPacketsReceived),
        )
        rowNumber = self.displayRow(rowNumber, self._odomLineOne)
        rowNumber = self.displayRow(rowNumber, self._odomLineTwo)
        rowNumber = self.displayRow(rowNumber, self._odomLineThree)
        rowNumber = self.displayRow(rowNumber, self._odomLineFour)
        rowNumber = self.displayRow(rowNumber, self._odomLineFive)
        if self._sendingTwistCommands:
            rowNumber = self.displayRow(rowNumber, "\tMoving around:")
            rowNumber = self.displayRow(rowNumber, "u\ti\to")
            rowNumber = self.displayRow(rowNumber, "j\tk\tl")
            rowNumber = self.displayRow(rowNumber, "m\t,\t.")
            rowNumber = self.displayRow(
                rowNumber, "a / z: increase / decrease max speeds by 10%"
            )
            rowNumber = self.displayRow(
                rowNumber, "s / x: increase / decrease only linear speed by 10%"
            )
            rowNumber = self.displayRow(
                rowNumber, "d / c: increase / decrease only angular speed by 10%"
            )
            rowNumber = self.displayRow(rowNumber, "q - quit sending twist commands")
        elif self._updatingSettings:
            rowNumber = self.displayRow(rowNumber, "\tUpdate Settings:")
            rowNumber = self.displayRow(
                rowNumber,
                "w - trackWidth decrease by 0.001\tW - trackWidth increase by 0.001",
            )
            rowNumber = self.displayRow(
                rowNumber,
                "d - distancePerCountDown decrease by 0.00001\tD - distancePerCountUp increase by 0.00001",
            )
            rowNumber = self.displayRow(
                rowNumber, "o / O: increase / decrease abd_speedLimit"
            )
            rowNumber = self.displayRow(
                rowNumber, "l / L: increase / decrease abdR_speedLimit%"
            )
            rowNumber = self.displayRow(
                rowNumber, "a - toggle ignore All proximity sensors"
            )
            rowNumber = self.displayRow(rowNumber, "c - toggle ignore Cliff sensors")
            rowNumber = self.displayRow(rowNumber, "r - toggle ignore IR sensors")
            rowNumber = self.displayRow(rowNumber, "f - toggle ignore Floor sensors")
            rowNumber = self.displayRow(rowNumber, "p - toggle Plugged in")
            rowNumber = self.displayRow(rowNumber, "q - Quit updating settings")
        elif self._performingTestManeuvers:
            rowNumber = self.displayRow(rowNumber, "\tTest Maneuvers:")
            rowNumber = self.displayRow(
                rowNumber, "f - Forward 1 meter\t\tb - Backward 1 meter"
            )
            rowNumber = self.displayRow(rowNumber, "4 - rotate 45 degrees")
            rowNumber = self.displayRow(rowNumber, "9 - rotate 90 degrees")
            rowNumber = self.displayRow(rowNumber, "8 - rotate 180 degrees")
            rowNumber = self.displayRow(rowNumber, "3 - rotate 360 degrees")
            rowNumber = self.displayRow(rowNumber, "r - reverse Rotation direction")
            # TODO: Options to customize distance and rotation amounts
            # TODO: Full operations, like "forward and back, "out and back", and "square" and other shapes.
            # TODO: maybe even the ability to make custom shapes,
            # TODO: Some help with dealing with the results of maneuver failures.
            rowNumber = self.displayRow(rowNumber, "i - Interrupt maneuvers")
            rowNumber = self.displayRow(
                rowNumber, "a / z: increase / decrease max speeds by 10%"
            )
            rowNumber = self.displayRow(
                rowNumber, "s / x: increase / decrease only linear speed by 10%"
            )
            rowNumber = self.displayRow(
                rowNumber, "d / c: increase / decrease only angular speed by 10%"
            )
            rowNumber = self.displayRow(rowNumber, "q - Quit test maneuvers")
        elif self._sendingLedCommands:
            # TODO: These are hard coded in the test, but you may or may not even have these.
            rowNumber = self.displayRow(rowNumber, "\tToggle LEDs:")
            rowNumber = self.displayRow(rowNumber, "0 - LED 0")
            rowNumber = self.displayRow(rowNumber, "1 - LED 1")
            rowNumber = self.displayRow(rowNumber, "2 - LED 2")
            rowNumber = self.displayRow(rowNumber, "3 - LED 3")
            rowNumber = self.displayRow(rowNumber, "4 - LED 4")
            rowNumber = self.displayRow(rowNumber, "q - Quit LED toggling")
        elif self._overridingPosition:
            if self._enteringPositionOverride is not None:
                rowNumber = self.displayRow(
                    rowNumber,
                    self._enteringPositionOverride + " " + self._positionOverrideData,
                )
            else:
                rowNumber = self.displayRow(rowNumber, "\tSet Robot Position:")
                rowNumber = self.displayRow(rowNumber, "x - X Position")
                rowNumber = self.displayRow(rowNumber, "y - Y Position")
                rowNumber = self.displayRow(rowNumber, "h - Heading")
                rowNumber = self.displayRow(rowNumber, "q - Quit Position Override")
        else:
            rowNumber = self.displayRow(rowNumber, "\tMain Menu:")
            rowNumber = self.displayRow(
                rowNumber,
                "t - send Test data | r - Run speed test | i - Interrupt speed test | m - send Move commands | s - Settings | a - mAneuvers | p - Position override | l - Led | q/Esc - Quit",
            )
        if rowNumber != self.headerHeight and self.height > 3:
            self.headerHeight = rowNumber
            self._scrollingOutputWindow = self.window.subwin(self.headerHeight, 0)

        if self._scrollingOutputWindow.getmaxyx()[0] > 2:

            if self._stayAtEnd:
                # If user has pressed END follow the last line
                if len(self.items) > self._max_lines():
                    self.top = len(self.items) - self._max_lines()
                    self.current = self._max_lines() - 1
                else:
                    self.current = len(self.items) - 1

            # Scrolling Output Window
            for idx, item in enumerate(
                self.items[self.top : self.top + self._max_lines()]
            ):
                # Highlight the current cursor line
                #   1 added to idx and x to offset for border
                if item:
                    stringStr = str(item)
                    stringStr = stringStr.replace("\n", " ").replace("\r", "")
                    try:
                        if idx == self.current:
                            self._scrollingOutputWindow.addstr(
                                idx + 1, 1, stringStr, curses.color_pair(2)
                            )
                        elif isinstance(idx, (int, long)):
                            self._scrollingOutputWindow.addstr(
                                idx + 1, 1, stringStr, curses.color_pair(1)
                            )
                    except TypeError:
                        pass
                        # TODO: I have no idea why this error happens sometimes.

            self._scrollingOutputWindow.border()

            self.window.addstr(
                self.headerHeight, 2, "Up/Down PgUp/PgDn Home/End(follow)"
            )

            windowFooter = (
                "Line: "
                + str(self.top + self.current + 1)
                + " of "
                + str(len(self.items))
            )
            if len(windowFooter) < self.width - 2:
                self.window.addstr(self.window.getmaxyx()[0] - 1, 2, windowFooter)
        self.window.refresh()


# noinspection PyUnusedLocal
def ignoreReturnData(data):
    return


def main():
    items = [str(num + 1) + ". Item" for num in range(995)]
    screen = Screen(items, ignoreReturnData)
    curses.wrapper(screen.run)


if __name__ == "__main__":
    main()
