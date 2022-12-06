xmin1, xmin2 = 5, 5
xmax1, xmax2 = 10, 10
ymin1, ymin2 = 5, 90
ymax1, ymax2 = 10, 95

minmaxrbt1 = [[xmin1, ymin1], [xmin1, ymax1], [xmax1, ymin1], [xmax1, ymax1]]
minmaxrbt2 = [[xmin2, ymin2], [xmin2, ymax2], [xmax2, ymin2], [xmax2, ymax2]]
minmax = [minmaxrbt1, minmaxrbt2]

print(minmax)
print(minmax[0])
print(minmax[0][0])
print(minmax[0][0][0])
print(minmax[0][0][1])

# # # # a = [0] * 5

# # # # for i in range(len(a)):
# # # #     print(a[i])
# # # from collections import defaultdict
# # # a = defaultdict()

# # # a[5] = 55
# # # a[6] = 66

# # # x = max(a.keys())
# # # print(x)
# # # print(a[x])


# # a = [11,22,33,44]
# # print(a)

# # res = a.pop(2)
# # print(a)

# # a.insert(2, res)
# # print(a)
# from collections import defaultdict
# a = defaultdict()

# try:
#     a[11] += 111
# except Exception:
#     a[11] = 222

# a[11] += 111

# print(a)
# # a[22] = 222
# # a[3] = 33

# # b = max(a.values())
# # print(b)
# # print((list(a.values())).index(b))
