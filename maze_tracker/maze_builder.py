RES = 4

# Initialise grid of symbols (each symbol represnts a type of junction/passage).
symbol_grid = []
for j in range(RES):
    row = []
    for i in range(RES):
        row.append('· ')
    symbol_grid.append(row)

# Build the symbol grid from user input.
# symbols = ('─', '│', '╢', '╤', '┐', '╟', '┌', '┬', '╧', '┘', '┤', '└', '┴', '├', '┼')
symbols = ('──', '│ ', '╢ ', '╤ ', '┐ ', '╟─', '┌─', '┬─', '╧ ', '┘ ', '┤ ', '└─', '┴─', '├─', '┼─')
finished = False
start_pos = None
end_pos = None
for j in range(RES):
    for i in range(RES):
        for k in range(RES):
            for l in range(RES):
                if (i, j) == (l, k):
                    print('◼ ', end='')
                elif [l, k] == start_pos:
                    print('s ', end='')
                elif [l, k] == end_pos:
                    print('e ', end='')
                else:
                    print(symbol_grid[l][k], end='')
            print()
        print(symbols[0:4])
        print()
        print(symbols[4:8])
        print()
        print(symbols[8:12])
        print()
        print(symbols[12:])
        user_input = input()
        if user_input == '/':
            finished = True
            break
        elif user_input == '':
            pass
        elif user_input[0] in ('s', 'e'):
            if user_input[0] == 's':
                start_pos = [i, j]
            else:
                end_pos = [i, j]
            user_input = user_input[1:]
        if user_input.isdigit():
            symbol_grid[i][j] = symbols[int(user_input)]
    if finished:
        break

for j in range(RES):
    for i in range(RES):
        print(symbol_grid[i][j], end='')
    print()