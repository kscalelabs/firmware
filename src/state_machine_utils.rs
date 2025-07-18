use futures_enum::Future;

/// Defines both the state-machine enum, its `Future` store, `StateTag`, and transition result.
#[macro_export]
macro_rules! state_machine {
    ( $( $variant:ident ),+ $(,)? ) => {
        paste::paste! {
            // 1) Combined future enum with per-state generic futures
            #[derive(::futures_enum::Future)]
            enum StateFutStore<$( [<$variant StateFut>] ),+> {
                $( $variant([<$variant StateFut>]), )+
            }

            // 2) Transition result
            pub struct StateTransitionResult {
                state: StateStore,
                result: ::std::io::Result<()>,
            }

            // 3) Opaque future alias
            type StateFut = impl ::std::future::Future<Output = StateTransitionResult>;

            // 4) Tag enum
            #[derive(::std::fmt::Debug, ::std::cmp::PartialEq, ::std::marker::Copy, ::std::clone::Clone)]
            pub enum StateTag {
                $( $variant, )+
            }

            // 5) Taggable trait and auto-impl for each state
            pub trait Taggable {
                fn tag(&self) -> StateTag;
            }
            $(
                impl Taggable for $variant {
                    fn tag(&self) -> StateTag {
                        StateTag::$variant
                    }
                }
            )+

            // 6) State trait depends on Taggable
            pub trait State: Taggable {
                fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult>;
            }

            // 7) Store enum
            #[derive(::std::fmt::Debug)]
            pub enum StateStore {
                $( $variant($variant), )+
            }

            // 8) Impl on store
            impl StateStore
            where
                $( $variant: State ),+
            {
                #[define_opaque(StateFut)]
                fn transition_fut(self) -> StateFut {
                    match self {
                        $( StateStore::$variant(st) => StateFutStore::$variant(st.transition_fut()), )+
                    }
                }

                fn tag(&self) -> StateTag {
                    match self {
                        $( StateStore::$variant(st) => st.tag(), )+
                    }
                }
            }
        }
    };
}
