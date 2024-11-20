clc;
clear;
close all;

%% 기존에 격투로봇 팔 해석에 사용한 2차원 R-Z 좌표계에서 RR 매니퓰레이터를 역기구학 해석한 코드를 활용했습니다.

link_lengths = [100, 100];
max_distance = sum(link_lengths); % 최대 작업 거리

square_points = [
    50, 50;  % 점 1
    50, 100; % 점 2
    100, 100;% 점 3
    100, 50; % 점 4
];

% 각 경로를 연결하는 샘플링 점 생성
trajectory_points = [];
for i = 1:size(square_points, 1)
    start_point = square_points(i, :);

    if i == size(square_points, 1)
        end_point = square_points(1, :); % 마지막 점에서 첫 번째 점으로 연결
    
    else
        end_point = square_points(i + 1, :);
    
    end

    r_vals = linspace(start_point(1), end_point(1), 50);
    z_vals = linspace(start_point(2), end_point(2), 50);
    trajectory_points = [trajectory_points; [r_vals', z_vals']];
end

for i = 1:size(trajectory_points, 1)
    try
        % 목표 위치 설정
        r = trajectory_points(i, 1);
        z = trajectory_points(i, 2);

        % 역기구학 계산
        [theta2, theta3] = inverseKinematics2D(r, z, link_lengths, max_distance);

        % 2D 시각화
        plotManipulator2D(theta2, theta3, link_lengths);

        pause(0.05); 
    catch ME
        disp(['오류: ', ME.message]);
    end
end

function [theta2, theta3] = inverseKinematics2D(r, z, link_lengths, max_distance)
    % 링크 길이
    link1 = link_lengths(1); 
    link2 = link_lengths(2); 

    % 목표 점까지의 직선 거리 계산
    distance = sqrt(r^2 + z^2);

    % 작업 공간 체크
    if distance > max_distance 
        error('입력한 위치는 작업 공간 바깥에 있습니다.');
    end

    % 각도 계산
    cos_theta2 = (r^2 + z^2 - (link1^2 + link2^2)) / (2 * link1 * link2);
    cos_theta2 = max(min(cos_theta2, 1), -1); % 코사인 값 제한
    theta3 = acos(cos_theta2);

    sin_theta2 = sqrt(1 - cos_theta2^2); % 삼각형의 높이 계산
    theta2 = atan2(z, r) - atan2(link2 * sin_theta2, link1 + link2 * cos_theta2);
end

function plotManipulator2D(theta2, theta3, link_lengths)
    % 링크 길이
    l2 = link_lengths(1);
    l3 = link_lengths(2); 

    x2 = l2 * cos(theta2);
    z2 = l2 * sin(theta2);

    x3 = x2 + l3 * cos(theta2 + theta3);
    z3 = z2 + l3 * sin(theta2 + theta3);

    figure(1); % 2D 시각화를 표시할 창
    clf; 
    hold on;
    grid on;

    % 축 설정
    axis equal;
    xlim([-200, 200]); % x축 범위
    ylim([-200, 200]); % z축 범위
    xlabel('r (mm)');
    ylabel('z (mm)');
    title('2D Manipulator Visualization');

    % 매니퓰레이터 플롯
    plot([0, x2], [0, z2], 'b-o', 'LineWidth', 2); 
    plot([x2, x3], [z2, z3], 'r-o', 'LineWidth', 2); 

    % 조인트 및 끝점 표시
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    plot(x2, z2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); 
    plot(x3, z3, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); 

    % 표시 업데이트
    drawnow;
end
