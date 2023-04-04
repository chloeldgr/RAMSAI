def work_on_gcode_file(sequence):
        length = len(sequence)

        delimiter = ';LAYER:'
        final_delimiter = '; Default end code'

        layers = [[]]

        for line in sequence:
                
                if (delimiter not in line) and (final_delimiter not in line):
                        if line[0]!=';':
                                layers[-1].append(line)
                else:
                        layers.append([])
        return layers

def getBordersGcode(sequence):
        zMax = 0
        xMax = 0
        yMax = 0
        zMin = 2000
        yMin = 2000
        xMin = 2000

        # remove initialisation

        completeSequence = work_on_gcode_file(sequence)
        innerSequence = []
        for i in range(1,len(completeSequence)-1):
                innerSequence = innerSequence+completeSequence[i]

        for line in innerSequence:
                if 'Z' in line and ';' not in line:
                        z = line.split('Z')[1]
                        z = z.split(' ')[0]
                        z = z.split('\n')[0]
                        z = float(z)
                        if z>zMax:
                                zMax = z
                        if z<zMin:
                                zMin = z
                if 'Y' in line and ';' not in line:
                        y = line.split('Y')[1]
                        y = y.split(' ')[0]
                        y = y.split('\n')[0]
                        y = float(y)
                        if y>yMax:
                                yMax = y
                        if y<yMin:
                                yMin = y
                if 'X' in line and ';' not in line:
                        x = line.split('X')[1]
                        x = x.split(' ')[0]
                        x = x.split('\n')[0]
                        x = float(x)
                        if x>xMax:
                                xMax = x
                        if x<xMin:
                                xMin = x
        return (xMin,yMin,zMin,xMax,yMax,zMax)