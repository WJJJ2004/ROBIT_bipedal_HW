clc;
clear;
close all;

%% 기본 설정
% 링크 길이 (mm 단위)
link_lengths = [36, 77.2, 100]; % 링크 1, 2, 3의 길이 (Base에서부터)
max_distance = sum(link_lengths(2:3)); % 링크 2와 링크 3의 최대 작업 거리

% 사용자로부터 목표 위치 입력받기
while true
    try
        % 목표 3D 평면 좌표 입력 (x, y, z)
        user_input = input('목표 3D 위치 (x, y, z)를 입력하세요 [x y z]: ', 's');
        if strcmpi(user_input, 'exit')
            disp('프로그램을 종료합니다.');
            break;
        end
        
        % 입력된 값을 숫자 배열로 변환
        target = str2num(user_input); %#ok<ST2NM>
        if numel(target) ~= 3
            error('x, y, z의 3개의 값을 입력해야 합니다.');
        end
        x = target(1); % x 좌표
        y = target(2); % y 좌표
        z = target(3); % z 좌표

        % Step 1: r 계산
        r = sqrt(x^2 + y^2); % XY 평면 거리
        if r == 0
            error('x와 y는 동시에 0일 수 없습니다.');
        end
        
        % Step 2: Z 방향 보정
        z_eff = z - link_lengths(1); % 링크 1 길이를 제외한 유효 z 값

        % Step 3: 2D 역기구학 계산 (Roll 축)
        [theta2, theta3] = inverseKinematics2D(r, z_eff, link_lengths(2:3), max_distance);

        % Step 4: Roll 축 계산
        theta1 = atan2(y, x); % 조인트 1의 회전각 (Yaw 축)

        % 결과 출력
        fprintf('조인트 각도 (degree):\n');
        fprintf('  조인트 1 (Yaw): %.2f°\n', rad2deg(theta1));
        fprintf('  조인트 2 (Roll): %.2f°\n', rad2deg(theta2));
        fprintf('  조인트 3 (Roll): %.2f°\n', rad2deg(theta3));

        % 3D 매니퓰레이터 시각화
        plotManipulator3D(theta1, theta2, theta3, link_lengths);
    catch ME
        disp(['오류: ', ME.message]);
    end
end

%% 역기구학 계산 함수 (2D)
function [theta2, theta3] = inverseKinematics2D(r, z, link_lengths, max_distance)
    % 링크 길이
    link2 = link_lengths(1); % 링크 2의 길이
    link3 = link_lengths(2); % 링크 3의 길이

    % 목표 점까지의 직선 거리 계산
    distance = sqrt(r^2 + z^2);

    % 작업 공간 체크
    if distance > max_distance 
        error('입력한 위치는 작업 공간 바깥에 있습니다.');
    end

    % 각도 계산
    cos_theta3 = (r^2 + z^2 - (link2^2 + link3^2)) / (2 * link2 * link3);
    cos_theta3 = max(min(cos_theta3, 1), -1); % 코사인 값 제한
    theta3 = acos(cos_theta3); % 조인트 3 (Roll 축)

    sin_theta3 = sqrt(1 - cos_theta3^2); % 삼각형의 높이 계산
    theta2 = atan2(z, r) - atan2(link3 * sin_theta3, link2 + link3 * cos_theta3); % 조인트 2 (Roll 축)
end

%% 3D 매니퓰레이터 시각화 함수
function plotManipulator3D(theta1, theta2, theta3, link_lengths)
    % 링크 길이
    l1 = link_lengths(1); % 링크 1의 길이
    l2 = link_lengths(2); % 링크 2의 길이
    l3 = link_lengths(3); % 링크 3의 길이

    % 3D 좌표 계산
    % 조인트 1 (Base에서 링크 1 끝)
    x1 = 0;
    y1 = 0;
    z1 = l1;

    % 조인트 2 위치 (링크 2 끝점, Roll 축 적용)
    x2 = l2 * cos(theta1) * cos(theta2);
    y2 = l2 * sin(theta1) * cos(theta2);
    z2 = z1 + l2 * sin(theta2);

    % 조인트 3 위치 (End Effector)
    x3 = x2 + l3 * cos(theta1) * cos(theta2 + theta3);
    y3 = y2 + l3 * sin(theta1) * cos(theta2 + theta3);
    z3 = z2 + l3 * sin(theta2 + theta3);

    % 3D 그래프 초기화
    figure(1); % 3D 시각화를 표시할 창
    clf; % 기존 그래프 초기화
    hold on;
    grid on;

    % 3D 축 설정
    axis equal;
    view(3); % 3D 뷰 활성화
    rotate3d on;
    xlim([-200, 200]); % x축 범위
    ylim([-200, 200]); % y축 범위
    zlim([-200, 200]);    % z축 범위
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    title('3D Manipulator Visualization');

    % 매니퓰레이터 플롯
    plot3([0, 0], [0, 0], [0, z1], 'k-o', 'LineWidth', 2); % 링크 1
    plot3([0, x2], [0, y2], [z1, z2], 'b-o', 'LineWidth', 2); % 링크 2
    plot3([x2, x3], [y2, y3], [z2, z3], 'r-o', 'LineWidth', 2); % 링크 3

    % 조인트 및 끝점 표시
    plot3(0, 0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Base
    plot3(x2, y2, z2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % 조인트 2
    plot3(x3, y3, z3, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % End Effector

    % 표시 업데이트
    drawnow;
end
