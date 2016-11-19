[y, Fs]=audioread('Audio_1.wav');
i=1;
j=1;
while i<length(y);
    if y(i)==0;
        count=i;
        while y(i) == 0 && i < length(y);
            i=i+1;
        end
        if i-count>500;
            silence_start(j)=count;
            silence_end(j)=i-1;
            j=j+1;
        end
    end
    i=i+1;
end
r=2;
while r<=length(silence_start);
    note_start(r)=silence_end(r-1);
    note_end(r)=silence_start(r);
    r=r+1;
end
note_start(1)=1;
note_end(1)=silence_start(1);
r=1;
while r<=j-1;
    sample_sound = y(note_start(r):note_end(r));
    y_fft = abs(fft(sample_sound));
    imax=find(y_fft == max(y_fft));
    fre(r)=Fs*(imax(1)-1)/length(sample_sound);
    r=r+1;
end
fre(1:j-1)'