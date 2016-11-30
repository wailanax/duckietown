function display(c)

disp(' ')
disp([inputname(1) ' ='])

disp(' ')

try

disp(['  ' type(c) ' - <a href="http://robotraconteur.com">Robot Raconteur</a> object reference'])

s=struct;

try
    lines1=textscan(members(c),'%s','delimiter','\n');
    lines=lines1{1};
    for i=1:length(lines)
        try
            line=lines{i};

            p1=textscan(line,'%s');
            p=p1{1};
            if (strcmp(p{1},'property'))
               S=struct;
               S.type='.';
               S.subs=p{3};
               s.(p{3})=subsref(c,S);
            end
        catch
        end
        
        
        
    end
catch
end

if (length(fieldnames(s))) > 0
   disp(' ')
   disp('  Properties:')
   disp(s) 
end
disp(['  <a href="matlab: members(' inputname(1) ')">Object Members</a>'])




catch
    disp('  Robot Raconteur object')
end
    disp(' ')
end



