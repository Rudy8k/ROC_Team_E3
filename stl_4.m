% Combined STL processing script with edge retracing and origin offset

% Replace 'your_stl_file.stl' with the path to your STL file
filename = 'link0.stl';
multiplier = 0.25;
tolerance = 20;
stepper = 20;

% Use stlread function to read the STL file
[TR,~] = stlread(filename);
faces = TR.ConnectivityList;
vertices = TR.Points;

% Set the desired origin offset
origin_offset = [10, 10, 0]; % Adjust these values as needed

% Apply the origin offset to all vertices
vertices = vertices + origin_offset;

% Initialize an empty edge list
edgeList = [];
edgeCount = containers.Map('KeyType','char','ValueType','any');
normalList = [];

% Function to compute the normal of a triangle
compute_normal = @(v1, v2, v3) cross(v2 - v1, v3 - v1);

% Loop through each face (triangle) and process edges
normals = zeros(size(faces, 1), 3);
for i = 1:size(faces, 1)
    vertexIndices = faces(i, :);
    
    v1 = vertices(vertexIndices(1), :);
    v2 = vertices(vertexIndices(2), :);
    v3 = vertices(vertexIndices(3), :);
    
    normal = compute_normal(v1, v2, v3);
    normal = normal / norm(normal);
    normals(i, :) = normal;
    
    edges = [vertexIndices(1), vertexIndices(2);
             vertexIndices(2), vertexIndices(3);
             vertexIndices(3), vertexIndices(1)];
    
    edges = sort(edges, 2);
    
    for j = 1:size(edges, 1)
        edgeKey = mat2str(edges(j, :));
        if isKey(edgeCount, edgeKey)
            normalList = edgeCount(edgeKey);
            isCoplanar = false;
            for k = 1:size(normalList, 1)
                if dot(normalList{k}, normal) > 0.9999
                    isCoplanar = true;
                    break;
                end
            end
            if isCoplanar
                edgeCount.remove(edgeKey);
            else
                edgeCount(edgeKey) = [normalList; {normal}];
            end
        else
            edgeCount(edgeKey) = {normal};
        end
    end
end

% Filter the edges
filteredEdges = [];
for k = keys(edgeCount)
    edge = str2num(k{1});
    filteredEdges = [filteredEdges; edge];
end

% Sort the filtered edges to remove duplicate edges
filteredEdges = sort(filteredEdges, 2);
filteredEdges = unique(filteredEdges, 'rows', 'stable');



% Function to check if two points are close enough
is_close = @(p1, p2) norm(p1 - p2) < tolerance;

% Rearrange the edges to form a continuous path with retracing
path = filteredEdges(1, :);
remaining_edges = filteredEdges(2:end, :);

while ~isempty(remaining_edges)
    current_point = path(end);
    found_next = false;
    
    % Check for exact match
    for i = 1:size(remaining_edges, 1)
        if remaining_edges(i, 1) == current_point
            path = [path, remaining_edges(i, 2)];
            remaining_edges(i, :) = [];
            found_next = true;
            break;
        elseif remaining_edges(i, 2) == current_point
            path = [path, remaining_edges(i, 1)];
            remaining_edges(i, :) = [];
            found_next = true;
            break;
        end
    end
    
    % If no exact match, check for close points within tolerance
    if ~found_next
        current_coords = vertices(current_point, :);
        for i = 1:size(remaining_edges, 1)
            point1 = vertices(remaining_edges(i, 1), :);
            point2 = vertices(remaining_edges(i, 2), :);
            
            if is_close(current_coords, point1)
                path = [path, remaining_edges(i, 2)];
                remaining_edges(i, :) = [];
                found_next = true;
                break;
            elseif is_close(current_coords, point2)
                path = [path, remaining_edges(i, 1)];
                remaining_edges(i, :) = [];
                found_next = true;
                break;
            end
        end
    end
    
    % If no close point found, add the nearest edge
    if ~found_next
        distances = min(pdist2(vertices(current_point, :), vertices(remaining_edges, :)), [], 2);
        [~, nearest_idx] = min(distances);
        path = [path, remaining_edges(nearest_idx, :)];
        remaining_edges(nearest_idx, :) = [];
    end
end

% Remove duplicate consecutive points in the path
path_final = path(1);
for i = 2:length(path)
    if path(i) ~= path_final(end)
        path_final = [path_final, path(i)];
    end
end

% Extract coordinates of the vertices in the final path
px = vertices(path_final, 1);
py = vertices(path_final, 2);
pz = vertices(path_final, 3);

% Interpolate and generate more points along the path

px_n = [];
py_n = [];
pz_n = [];

for i = 1:length(px)-1
    for j = 1:stepper
        px_n = [px_n; px(i) + (j-1) * (px(i+1) - px(i)) / stepper];
        py_n = [py_n; py(i) + (j-1) * (py(i+1) - py(i)) / stepper];
        pz_n = [pz_n; pz(i) + (j-1) * (pz(i+1) - pz(i)) / stepper];
    end
end

% Scale the coordinates

px_n = px_n * multiplier;
py_n = py_n * multiplier;
pz_n = pz_n * multiplier;

% Prepare final output
p = [px_n, py_n, pz_n, zeros(length(px_n), 3)];

% Save the variable 'p' to a .csv file
csv_filename = 'final.csv';
writematrix(p, csv_filename);

% Plotting
figure;
hold on;

% Plot the outline
for i = 1:length(path_final)-1
    v1 = path_final(i);
    v2 = path_final(i+1);
    plot3([vertices(v1, 1), vertices(v2, 1)], ...
          [vertices(v1, 2), vertices(v2, 2)], ...
          [vertices(v1, 3), vertices(v2, 3)], 'b-', 'LineWidth', 2);
end

% Plot the interpolated path
plot3(px_n, py_n, pz_n, 'r-', 'LineWidth', 1);

% Plot the origin
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

title('STL Object Outline and Interpolated Path');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
rotate3d on;
hold off;