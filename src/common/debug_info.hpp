 #include <iostream>
 template <typename... Tn> void DebugInfo(Tn... tn)
    {
        const int len_f = sizeof...(tn);
        int cnt = 0;
        auto f = [&](auto it) {
            cnt == 0 ? std::cout << it << ":\t" : std::cout << it << " " << std::endl;
            cnt ^= 1;
        };

        (..., f(tn));//一元素左折叠
    }

