my_list = ['a-1', 'b-2', 'c-3', 'd-4']

# ✅ split each element in list and keep first part
result_1 = [item.split('-', 1)[0] for item in my_list]
print(result_1)  # 👉️ ['a', 'b', 'c', 'd']

# ✅ split each element in list into nested lists
result_2 = [item.split('-') for item in my_list]
print(result_2)  # 👉 [['a', '1'], ['b', '2'], ['c', '3'], ['d', '4']]

# ✅ split each element in list and flatten list
result_3 = [item.split('-') for item in my_list]

result_3_flat = [item for l in result_3 for item in l]
print(result_3_flat)  # 👉️ ['a', '1', 'b', '2', 'c', '3', 'd', '4']

# ✅ split specific element in list
result_4 = my_list[0].split('-')
print(result_4)  # 👉️ ['a', '1']
