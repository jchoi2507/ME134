
 function write_J(J_qs)
     write_site = "http://192.168.4.1/test";
     count = length(J_qs);
     for i = 1:count
         for j = 1:3
             response = webwrite(site_name, J_qs(i, j));
             pause(0.1);
         end
         pause(0.5);
     end
 end

 function write_W(W_qs)
     write_site = "http://192.168.4.1/test";
     count = length(W_qs);
     for i = 1:count
         for j = 1:3
             response = webwrite(site_name, W_qs(i, j));
             pause(0.1);
         end
         pause(0.5);
     end
  end

   function write_G(G_qs)
     write_site = "http://192.168.4.1/test";
     count = length(G_qs);
     for i = 1:count
         for j = 1:3
             response = webwrite(site_name, G_qs(i, j));
             pause(0.1);
         end
         pause(0.5);
     end
 end


function write_C(C_qs)
    write_site = "http://192.168.4.1/test";
    count = length(C_qs);
    for i = 1:count
        for j = 1:3
            response = webwrite(site_name, C_qs(i, j));
            pause(0.1);
        end
        pause(0.5);
    end
end