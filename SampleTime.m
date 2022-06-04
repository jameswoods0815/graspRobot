%time interpelation
function output = SampleTime(totalTime, nowTime)
    a1 = 10/totalTime^3;
    a2 = -15/totalTime^4;
    a3 = 6/totalTime^5;
    output = a1*nowTime^3 + a2*nowTime^4 + a3*nowTime^5;
end