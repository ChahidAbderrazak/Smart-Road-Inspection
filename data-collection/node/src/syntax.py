import ast
original_String = '{"John" : 01, "Rick" : 02, "Sam" : 03}'

# printing original string
print("The original string is : " + str(original_String))

# using ast.literal_eval() method
result = ast.literal_eval(original_String)

# print result
print("The converted dictionary is : " + str(result))