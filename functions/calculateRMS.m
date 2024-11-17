function error = calculateRMS(vector, window_size)

    % Initialize the RMS array
    %rms_value = zeros(1, length(vector) - window_size + 1);
    
    % Calculate RMS value for each window
    rms_value = [];

    for i = 1:window_size:length(vector)
        % Extract the window of data
        idx = i + window_size - 1;
        if idx >length(rms_value)
            window_data = vector(i:end);
        else
            window_data = vector(i:idx);
        end
        
        
        % Calculate the RMS value of the window
        %rms_value(i) = sqrt(mean(window_data.^2));
        rms_value = [rms_value,rms(window_data)];
    end

    error = rms_value;%rms_value(1:window_size:end);
    return
end