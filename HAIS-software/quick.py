points_list=[(-1,23),(-1,2),(4,-1)]
print(points_list)
def rectify_route_positions(points_list):
    return [(i,j) for (i,j) in points_list if i!=-1 and j!=-1]
    
points_list= rectify_route_positions(points_list)
print(points_list)