;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((nil . ((lsp-file-watch-ignored . (;; SCM tools
                                    "[/\\\\]\\.git$"
                                    "[/\\\\]\\.hg$"
                                    "[/\\\\]\\.bzr$"
                                    "[/\\\\]_darcs$"
                                    "[/\\\\]\\.svn$"
                                    "[/\\\\]_FOSSIL_$"
                                    ;; IDE tools
                                    "[/\\\\]\\.idea$"
                                    "[/\\\\]\\.ensime_cache$"
                                    "[/\\\\]\\.eunit$"
                                    "[/\\\\]node_modules$"
                                    "[/\\\\]\\.fslckout$"
                                    "[/\\\\]\\.tox$"
                                    "[/\\\\]\\.stack-work$"
                                    "[/\\\\]\\.bloop$"
                                    "[/\\\\]\\.metals$"
                                    "[/\\\\]target$"
                                    "[/\\\\]\\.ccls-cache$"
                                    "[/\\\\]\\.catkin_tools$"
                                    ;; Autotools output
                                    "[/\\\\]\\.deps$"
                                    "[/\\\\]build-aux$"
                                    "[/\\\\]autom4te.cache$"
                                    "[/\\\\]\\.reference$"
                                    ;; catkin_tools
                                    "[/\\\\]build$"
                                    "[/\\\\]donotbuild$"
                                    "[/\\\\]logs$"
                                    "[/\\\\]devel$")))))
