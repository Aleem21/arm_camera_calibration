function transformations = read_transformations(filename)
    file = fopen(filename, "r");
    transformations = fscanf(file, "%f", Inf);
    transformations = reshape(transformations, 4, 4, rows(transformations) / 16);
end

